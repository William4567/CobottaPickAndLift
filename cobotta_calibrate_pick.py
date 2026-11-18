"""COBOTTA: one RoboDK Python entry point for calibration, preview and pick/lift.

Run from the station tree. No application/server, no robot driver connection.
Imports are passive; hardware is accessed only after main() selects an operation.
Development does not run this entry point; user-run evidence is saved in sessions/.
All distances are mm, angles degrees unless explicitly named radians.
T_A_B maps coordinates expressed in B into A. F is physical J6; M is stock J5.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import shutil
import signal
import sys
import time
import traceback
from contextlib import contextmanager
from datetime import datetime, timezone

# One configuration file holds every user setting and the run state (saved
# VIEW, accepted calibration binding). It lives in the repository folder next
# to this script so the user can edit it. It is tracked with placeholder values
# because it holds the camera password: fill it in, do not commit it, and the
# program never creates it. RoboDK may run a temporary copy of this script, so
# the folder is located by checking this script's folder, an optional override
# and a short search of the usual clone locations under the user profile.
PROJECT_DIRECTORY = ""   # Optional override; set only if the search below fails.
CONFIG_NAME = "config.json"
CONFIG_TEMPLATE = {
    "robot_ip": "ROBOT_IP",
    "camera_ip": "CAMERA_IP",
    "camera_username": "CAMERA_USER",
    "camera_password": "CAMERA_PASSWORD",
    "approach_clearance_mm": 40,
    "lift_mm": 30,
    "speed_percent": 100,
    "observation": {"pose": None, "joints": None, "tool": 0, "work": 0},
    "vision": {"roi_xywh": None, "hsv_lower": [90, 40, 15],
               "hsv_upper": [135, 255, 255], "min_area_px": 400,
               "max_area_px": 200000, "max_aspect_ratio": 1.6,
               "min_rectangular_fill": 0.7},
}
PROJECT_FILES = ("bcapclient.py", "variant.py", "orinexception.py", "http_camera.py",
                 "detect_blue.py", "cobotta_calibrate_pick.py", "automatic_setup.json",
                 "config.json", "charuco-7x5.pdf", "charuco-7x5-letter.pdf",
                 "collision_assets/factory_camera_J5_envelope.stl",
                 "collision_assets/stock_gripper_body_F_envelope.stl")


def _stop(message):
    """Stop before any hardware work, with text the operator can see.

    RoboDK does not display the text of a bare SystemExit raised at import, so
    the reason is printed first and then raised as an ordinary error.
    """
    print(message, file=sys.stderr, flush=True)
    raise RuntimeError(message)


def _find_project_directory():
    candidates = []
    if PROJECT_DIRECTORY.strip():
        candidates.append(Path(PROJECT_DIRECTORY).expanduser())
    candidates += [Path(__file__).resolve().parent, Path.cwd()]
    home = Path.home()
    roots = [(home, 1), (home/"Documents", 2), (home/"Desktop", 2),
             (home/"Downloads", 2), (home/"source", 2), (home/"repos", 2),
             (home/"GitHub", 2), (home/"git", 2),
             (home/"OneDrive"/"Documents", 2), (home/"OneDrive"/"Desktop", 2),
             (home/"Documents"/"GitHub", 1), (home/"source"/"repos", 1)]
    # School and business accounts sync as "OneDrive - <organisation>" and a
    # clone can sit four folders below that root, past the bounded walk.
    for cloud in sorted(home.glob("OneDrive*")):
        if cloud.is_dir():
            roots.append((cloud, 4))
    for root, depth in roots:
        if not root.is_dir():
            continue
        patterns = ["cobotta_calibrate_pick.py"]
        for level in range(1, depth + 1):
            patterns.append("/".join(["*"]*level) + "/cobotta_calibrate_pick.py")
        for pattern in patterns:
            try:
                for script in root.glob(pattern):
                    candidates.append(script.parent)
            except OSError:
                continue
    # Last resort: a bounded walk of the user profile, so a clone anywhere
    # under the user folder is found. Skip the large or irrelevant folders.
    skip = {"AppData", ".git", ".hg", ".svn", "node_modules", "__pycache__",
            "sessions", "calibrations", "backups", "vendor", "venv", ".venv"}
    for base, dirs, files in os.walk(home, onerror=lambda exc: None):
        try:
            depth = len(Path(base).relative_to(home).parts)
        except ValueError:
            continue
        dirs[:] = [name for name in dirs if name not in skip and not name.startswith(".")]
        if depth >= 4:
            dirs[:] = []
        if "cobotta_calibrate_pick.py" in files:
            candidates.append(Path(base))
    seen, found = set(), []
    for folder in candidates:
        try:
            folder = folder.resolve()
        except OSError:
            continue
        if folder in seen:
            continue
        seen.add(folder)
        if all((folder/name).is_file() for name in PROJECT_FILES):
            found.append(folder)
    if len(found) == 1:
        return found[0]
    if found:
        _stop("Several copies of this project were found:\n  " +
              "\n  ".join(str(folder) for folder in found) +
              "\nSet PROJECT_DIRECTORY near the top of cobotta_calibrate_pick.py to the "
              "copy you want to use.")
    _stop("COBOTTA project folder not found. Clone the repository under your user folder "
          "(for example Documents) and run again, or set PROJECT_DIRECTORY near the top of "
          "cobotta_calibrate_pick.py to the clone path. Searched under:\n  " +
          "\n  ".join(str(folder) for folder in [Path(__file__).resolve().parent,
                                                Path.cwd(), home]))


def _config_help(fields):
    return ("Edit the configuration file and run again:\n  " + str(CONFIG_FILE) +
            "\nFields to set: " + ", ".join(fields) +
            "\nSee guide.md for what each field means.")


def _load_config():
    if not CONFIG_FILE.is_file():
        _stop("config.json is missing from the project folder:\n  " + str(CONFIG_FILE) +
              "\nRestore the placeholder copy from the repository with:\n  "
              "git checkout -- config.json\nThen edit it (see guide.md).")
    try:
        config = json.loads(CONFIG_FILE.read_text(encoding="utf-8-sig"))
    except ValueError as exc:
        _stop("Could not read " + str(CONFIG_FILE) + ": " + str(exc))
    for key, value in CONFIG_TEMPLATE.items():
        if key not in config:
            config[key] = value
    if not isinstance(config.get("observation"), dict):
        config["observation"] = dict(CONFIG_TEMPLATE["observation"])
    for key, value in CONFIG_TEMPLATE["observation"].items():
        config["observation"].setdefault(key, value)
    if not isinstance(config.get("vision"), dict):
        config["vision"] = dict(CONFIG_TEMPLATE["vision"])
    for key, value in CONFIG_TEMPLATE["vision"].items():
        config["vision"].setdefault(key, value)
    placeholders = {"robot_ip": "ROBOT_IP", "camera_ip": "CAMERA_IP",
                    "camera_username": "CAMERA_USER",
                    "camera_password": "CAMERA_PASSWORD"}
    missing = [key for key, value in placeholders.items()
               if not config.get(key) or str(config.get(key)) == value]
    if missing:
        _stop(_config_help(missing))
    return config


ROOT = _find_project_directory()
CONFIG_FILE = ROOT/CONFIG_NAME
CONFIG = _load_config()
# Optional, user-installed dependency folder (no administrator rights):
#   python -m pip install --target <clone>\vendor numpy==1.25.2 opencv-contrib-python==4.5.5.64
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/"vendor"))

import cv2
import numpy as np
from robodk import robolink, robomath, robodialogs

from bcapclient import BCAPClient
from http_camera import HTTPCamera
from detect_blue import blue_mask, detect, overlay

SCHEMA = "cobotta.eye_in_hand.v2"
PROGRAM_REVISION = "2026-11-13-fast-planning-r8"
CAMERA_ENVELOPE = "collision_assets/factory_camera_J5_envelope.stl"
CAMERA_ENVELOPE_SHA256 = "9daf6e4b13f3ce0ee7afc1d5ad41ca06ea1fb8390593dec2502ebc0fbceb229d"
GRIPPER_BODY_ENVELOPE = "collision_assets/stock_gripper_body_F_envelope.stl"
GRIPPER_BODY_SHA256 = "bed7a9debe3d7fc1472cc8debd6eeb71a8dd7ece4d53fd466523df44d09e47ba"
STOP = False
STOP_REASON = None
RDK = None
WINDOW = "COBOTTA camera images - Esc/Q stops"
# check_stop() runs inside every collision sample, so keyboard/file polling is
# throttled. Displayed frames still pump the GUI on every show_image() call.
_STOP_CHECK_SECONDS = 0.1
_LAST_STOP_CHECK = 0.0
_WINDOW_READY = False
OPERATIONS = ("calibrate", "preview", "pick", "status", "board", "instructions", "set-view")
# User-supplied CalibrX PDF: footer dimensions and all 17 marker IDs read from
# the document. The top-left square is black; marker IDs run 0..16 row-major.
BOARD = dict(squares_x=7, squares_y=5, square_mm=25.0, marker_mm=18.0,
             dictionary="DICT_4X4_50", opencv_layout="4.5.5", width_mm=175.0,
             height_mm=125.0, marker_ids=list(range(17)),
             source_pdf="charuco-7x5.pdf",
             source_pdf_sha256="339e5548913dd1d944dcbfebf1c06952ec2074a93cad5baf5e799e64a61c6801")
# Same physical pattern as the source PDF, translated onto a US Letter page.
# Page layout is separate from BOARD so repaging does not change calibration geometry.
PRINTABLE_BOARD = "charuco-7x5-letter.pdf"
PRINTABLE_BOARD_SHA256 = "5804a81a5c1cc8120bdb285e5ab68bf2e59e259030bea72203452bbc888e284d"
# Read from the hash-locked Letter PDF's image transform. OpenCV 4.5.5
# board origin is the pattern's lower-left, X right and Y toward its top.
# This registration is used only after the operator confirms the physical print.
PRINTED_PAPER_LAYOUT = {"size_mm": [215.9, 279.4],
                        "pattern_lower_left_from_page_lower_left_mm": [20.45, 81.2]}
# Fixed calibration workflow. Hardware dimensions, limits and verified board/
# optics settings are in automatic_setup.json. Direction was confirmed in
# session 20261104T215150257604Z_calibrate for the fixed Letter-paper placement.
FIXED_CALIBRATION = {
    "default_mode": "calibrate",
    "toward_user_board_xy": (0.0, 1.0),
    "expected_toward_user_base_xy": (1.0, 0.0),
    "paper_direction_tolerance_deg": 5.0,
    "snapshot_timeout_seconds": 10,
}
# The measured approximately 25 mm part has rounded edges and side slots.
# Permit their small contour deficits; the independent 3D size/pose fit below
# must still pass before planning. Colour comes from the robot camera config.
PICK_OBJECT_PROFILE = {"name": "rounded_perforated_cube", "min_outline_solidity": .94,
                       "max_fit_rms_mm": 1.0, "min_silhouette_iou": .85,
                       "visible_surface": "top_face_at_object_height",
                       "glare_repair": {"min_blue_fraction": .30, "max_saturation": 40,
                                        "max_value": 100, "background_value_gap": 30}}
# A uniform 1 mm inset on a 25 mm square gives (23/25)^2 = .846 IoU, below
# the 0.85 silhouette gate, so the gate tolerates roughly 0.9 mm of uniform
# inset. The calibrated 1 mm RMS check separately rejects size discrepancies.
SPEED_VARS = ("@EXTSPEED", "@EXTACCEL", "@EXTDECEL", "@SPEED", "@ACCEL",
              "@DECEL", "@JSPEED", "@JACCEL", "@JDECEL")
# Percent. A run must not start with the pendant override muting the arm: the
# 2026-11-13 pick ran at @EXTSPEED=1.0 / @EXTACCEL=0.01 / @EXTDECEL=0.01, crawled
# for about two minutes and then stopped when the override changed. These values
# are only ever read here; this program never writes an external override.
EXTERNAL_OVERRIDE_FLOOR = 5.0


class TimingCollector(dict):
    """Coarse phase seconds, accumulated with time.perf_counter().

    Diagnostics only: adding a phase must never change control flow or a return
    value, and the accumulated numbers are written to <session>/timings.json.
    """
    def add(self, name, seconds):
        self[name] = self.get(name, 0.0) + float(seconds)

    @contextmanager
    def phase(self, name):
        started = time.perf_counter()
        try:
            yield
        finally:
            self.add(name, time.perf_counter()-started)


TIMINGS = TimingCollector()


def save_timings(folder):
    # Never mask the original exception or cancel: failures are logged only.
    try:
        write_json(Path(folder)/"timings.json",
                   {"program_revision": PROGRAM_REVISION, "session": Path(folder).name,
                    "clock": "seconds accumulated with time.perf_counter over this process",
                    "seconds": {name: round(seconds, 3)
                                for name, seconds in sorted(TIMINGS.items())}})
    except Exception as exc:
        try:
            log("Timings could not be written (ignored): " + str(exc))
        except Exception:
            pass


class Stopped(RuntimeError):
    pass


class NoIKSolution(ValueError):
    pass


def require(value, message):
    if not value:
        raise ValueError(message)


def stamp():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, data, backup=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(data, indent=2, allow_nan=False)
    tmp = path.with_name(path.name + "." + stamp() + ".tmp")
    tmp.write_text(text + "\n", encoding="utf-8")
    if backup and path.exists():
        shutil.copy2(path, path.with_name(path.stem + ".backup_" + stamp() + path.suffix))
    os.replace(tmp, path)


def fingerprint(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


def board_pdf_matches(path):
    """True when the printable board is the supplied PDF.

    Git for Windows installs with core.autocrlf=true, and a clone made by such
    a Git can hold the PDF with CRLF instead of LF. That is the same document,
    so a converted copy is compared after undoing the conversion; every other
    change still fails the shipped hash.
    """
    data = Path(path).read_bytes()
    if hashlib.sha256(data).hexdigest() == PRINTABLE_BOARD_SHA256:
        return True
    return hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest() == PRINTABLE_BOARD_SHA256


def array(value, shape, label):
    a = np.asarray(value, dtype=float)
    require(a.shape == shape and np.isfinite(a).all(), "Invalid/missing " + label)
    return a


def transform(r=None, t=None):
    h = np.eye(4)
    if r is not None:
        h[:3, :3] = r
    if t is not None:
        h[:3, 3] = np.asarray(t).reshape(3)
    return h


def grasp_transform(setup):
    # Current object geometry, independent of the camera calibration. Keep the
    # physical finger tips at the configured clearance above the table.
    z = setup["finger_tip_z_mm"]-(setup["cube_height_mm"]/2-setup["tip_clearance_mm"])
    return transform(t=[0, 0, z])


def rigid(h):
    h = array(h, (4, 4), "rigid transform")
    require(np.allclose(h[3], [0, 0, 0, 1], atol=1e-7) and
            np.allclose(h[:3, :3].T @ h[:3, :3], np.eye(3), atol=1e-5) and
            abs(np.linalg.det(h[:3, :3])-1) < 1e-5, "Non-rigid calibration transform")
    return h


def inv(h):
    return transform(h[:3, :3].T, -h[:3, :3].T @ h[:3, 3])


def points(h, p):
    return np.asarray(p) @ h[:3, :3].T + h[:3, 3]


def pose_matrix(p):
    # DENSO Rx/Ry/Rz: Rz(Rz) Ry(Ry) Rx(Rx), translation in Work 0.
    return np.asarray(robomath.xyzrpw_2_pose(list(map(float, p[:6]))).rows)


def documented_physical_fk(q):
    """Stock COBOTTA J6 mounting frame, from DENSO's public robot description.

    Source: DENSORobot/denso_cobotta_ros, cobotta.urdf.xacro (metres converted
    to mm). Used to check reference conventions, not to replace controller limits.
    """
    origins = [[0, 0, 0], [0, 0, 180], [0, 0, 165],
               [-12, 20, 88], [0, -20, 89.5], [0, -44.5, 42]]
    axes = [[0, 0, 1], [0, 1, 0], [0, 1, 0],
            [0, 0, 1], [0, 1, 0], [0, 0, 1]]
    h = np.eye(4)
    for origin, axis, degrees in zip(origins, axes, q):
        r = cv2.Rodrigues(np.asarray(axis, float)*math.radians(float(degrees)))[0]
        h = h @ transform(t=origin) @ transform(r)
    return h


def camera_mount_fk(q):
    # The factory camera is on J5, BEFORE J6 rotation. This is a different
    # kinematic frame from both the physical flange and the stock fingertip.
    q = array(q, (6,), "camera mounting-link joints")
    return documented_physical_fk(q[:5])


def rotation_vector(r):
    return cv2.Rodrigues(r)[0].reshape(3)


def rotation_error(a, b):
    return float(np.linalg.norm(rotation_vector(a[:3, :3].T @ b[:3, :3]))*180/math.pi)


def viewing_tilt(a, b):
    """Out-of-plane tilt in degrees between two camera-mount poses.

    The camera looks along the mount z axis (1.2 deg in the accepted
    calibration), so the angle between those axes is the board tilt the fit
    sees. In-plane roll, which counts in rotation_error(), does not count here:
    it leaves the intrinsics as weakly constrained as a translation.
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    return math.degrees(math.acos(float(np.clip(np.dot(a[:3, 2], b[:3, 2]), -1.0, 1.0))))


def mean_transform(hs):
    u, _, vt = np.linalg.svd(sum(h[:3, :3] for h in hs))
    r = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return transform(r, np.mean([h[:3, 3] for h in hs], axis=0))


def log(message):
    print(message, flush=True)
    if RDK is not None:
        try:
            RDK.ShowMessage(str(message), False)
        except Exception as exc:
            # Losing the simulator must not interrupt HALT/GiveArm cleanup or
            # replace the original controller/capture failure.
            print("RoboDK message unavailable: " + str(exc), file=sys.stderr, flush=True)


def confirm(message):
    if not robodialogs.ShowMessageOkCancel(message, title="COBOTTA Python"):
        raise Stopped("Cancelled. No further movements.")


def stop_signal(number, *_):
    global STOP, STOP_REASON
    STOP = True
    STOP_REASON = "signal " + signal.Signals(number).name


def check_stop(pump=False):
    global STOP, STOP_REASON, _LAST_STOP_CHECK
    if STOP:
        raise Stopped("Stop requested (%s). No automatic return, retry or release." %
                      (STOP_REASON or "unspecified source"))
    now = time.monotonic()
    if not pump and now - _LAST_STOP_CHECK < _STOP_CHECK_SECONDS:
        return
    _LAST_STOP_CHECK = now
    key = cv2.waitKey(1) & 0xFF
    if key in (27, ord("q")):
        STOP = True
        STOP_REASON = "Esc in camera window" if key == 27 else "Q in camera window"
    if (ROOT / "STOP_COBOTTA").exists():
        STOP = True
        STOP_REASON = str(ROOT / "STOP_COBOTTA")
    if STOP:
        raise Stopped("Stop requested (%s). No automatic return, retry or release." %
                      (STOP_REASON or "unspecified source"))


def pause(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        check_stop()
        time.sleep(0.04)


def show_image(frame, label):
    global _WINDOW_READY
    frame = frame.copy()
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 65), (0, 0, 0), -1)
    cv2.putText(frame, label[:135], (12, 38), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (255, 255, 255), 2)
    if not _WINDOW_READY:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1152, 648)
        _WINDOW_READY = True
    cv2.imshow(WINDOW, frame)
    check_stop(pump=True)
    return frame


def get_board():
    require(cv2.__version__.startswith("4.5.5"),
            "This implementation uses OpenCV 4.5.5 coordinates for the supplied CalibrX "
            "7x5 sheet. Use the existing RoboDK Python with opencv-contrib 4.5.5; "
            "do not silently substitute coordinate conventions.")
    require(hasattr(cv2, "aruco"), "OpenCV contrib (aruco) is required.")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, BOARD["dictionary"]))
    board = cv2.aruco.CharucoBoard_create(BOARD["squares_x"], BOARD["squares_y"],
                                        BOARD["square_mm"], BOARD["marker_mm"], dictionary)
    require(board.ids.reshape(-1).astype(int).tolist() == BOARD["marker_ids"],
            "OpenCV marker ID layout differs from the supplied PDF")
    return board, dictionary


def validate_setup(setup, for_calibration):
    require(setup.get("stock_fingers_confirmed") is True,
            "Fixed setup requires the original unmodified DENSO fingers; verify automatic_setup.json")
    require(setup.get("factory_camera_mount_confirmed") is True,
            "This geometry requires the original Canon N10-W02 camera, factory bracket "
            "and original cable routing. A relocated assembly needs new geometry.")
    require(setup.get("camera_mount_frame") == "DENSO_ROS_J5" and
            setup.get("camera_envelope_sha256") == CAMERA_ENVELOPE_SHA256 and
            hashlib.sha256((ROOT/CAMERA_ENVELOPE).read_bytes()).hexdigest() == CAMERA_ENVELOPE_SHA256,
            "Factory camera collision asset/frame does not match this program")
    require(hashlib.sha256((ROOT/GRIPPER_BODY_ENVELOPE).read_bytes()).hexdigest() == GRIPPER_BODY_SHA256,
            "Stock gripper collision asset does not match this program")
    require(float(setup["planning"]["margin_mm"]) == 2.0,
            "The supplied factory camera/gripper meshes include a fixed 2 mm allowance. "
            "A different margin requires regenerated verified assets; no margin was changed.")
    if for_calibration:
        require(setup.get("board_scale_verified") is True,
                "automatic_setup.json does not record the verified printed-board scale; "
                "restore the supplied file")
        require(setup.get("board_definition_fingerprint") == fingerprint(BOARD),
                "automatic_setup.json board definition does not match this program's "
                "CalibrX 7x5 definition; restore the supplied file")
        require(board_pdf_matches(ROOT/PRINTABLE_BOARD),
                "The supplied charuco-7x5-letter.pdf is missing or changed (expected "
                "sha256 " + PRINTABLE_BOARD_SHA256 + "); restore it with:\n  "
                "git checkout HEAD -- charuco-7x5.pdf charuco-7x5-letter.pdf")
        thickness = setup.get("board_thickness_mm")
        require(isinstance(thickness, (int, float)) and 0 <= thickness <= 20 and
                setup.get("optics_locked_confirmed") is True,
                "Fixed setup requires measured board thickness and locked optics in automatic_setup.json")
    require(setup["base_on_same_table"], "A measured mounting/table transform is required")
    require(6 <= setup["gripper"]["force_n"] <= 20, "Stock gripper force must be 6-20 N")
    require(0 < setup["tip_clearance_mm"] < setup["cube_height_mm"]/2,
            "Tip clearance must leave contact below the cube centre")


class Controller:
    """Existing BCAPClient transport, with confirmed startup motor-on only.

    No fault reset, CALSET, limit changes or external-override writes.
    """
    def __init__(self, cfg, setup):
        self.cfg, self.setup = cfg, setup
        self.client = self.ctrl = self.arm = None
        self.started = self.owned = False
        self.initial_frames = None
        self.initial_external = None
        self.holding_required = False
        self.T_flange_controller = None
        self.reference_report = None
        self.variable_handles = {}
        # Preflight-only memos of the pure OutRange and J2P queries.
        self.range_cache = {}
        self.pose_cache = {}

    def __enter__(self):
        try:
            self.client = BCAPClient(self.cfg["robot_ip"], self.setup["robot_port"], 3.0)
            self.client.service_start("")
            self.started = True
            self.ctrl = self.client.controller_connect("", "CaoProv.DENSO.VRC", "localhost", "")
            self.arm = self.client.controller_getrobot(self.ctrl, "Arm", "")
            self.initial_frames = self.frames()
            self.initial_external = [float(self.var(k)) for k in SPEED_VARS[:3]]
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def var(self, name):
        # Cache object handles for this connection, NEVER live values. DENSO's
        # b-CAP lifecycle permits repeated GetValue calls until Variable_Release.
        if name not in self.variable_handles:
            self.variable_handles[name] = self.client.robot_getvariable(self.arm, name, "")
        return self.client.variable_getvalue(self.variable_handles[name])

    def call(self, command, args=None):
        return self.client.robot_execute(self.arm, command, args)

    def hand(self, command, args=None):
        return self.client.controller_execute(self.ctrl, command, args)

    def frames(self):
        return {"tool": self.var("@CURRENT_TOOL"), "work": self.var("@CURRENT_WORK"),
                "tool0": self.var("TOOL0"), "work0": self.var("WORK0")}

    def check_frames(self):
        f = self.frames()
        require(f["tool"] == 0 and f["work"] == 0, "Select Tool 0 / Work 0 using normal controls")
        require(np.allclose(np.asarray(f["tool0"])[:6], 0, atol=1e-7) and
                np.allclose(np.asarray(f["work0"])[:6], 0, atol=1e-7),
                "Tool 0 and Work 0 must retain zero user offsets")
        require(f == self.initial_frames, "Tool/Work definitions changed; stopped")

    def state(self):
        result = {"joints": list(self.var("@CURRENT_ANGLE"))[:6],
                "pose": list(self.var("@CURRENT_POSITION")),
                "busy": bool(self.var("@BUSY_STATUS")),
                "servo": bool(self.var("@SERVO_ON")),
                "error": self.var("@ERROR_CODE"),
                "hand_busy": bool(self.hand("HandBusyState")),
                "hand_holding": bool(self.hand("HandHoldState")),
                "hand_position": float(self.hand("HandCurPos"))}
        # Only pay for the description round trip when the controller reports
        # a nonzero error code.
        result["error_description"] = self.var("@ERROR_DESCRIPTION") if result["error"] else ""
        if self.T_flange_controller is not None:
            result["T_base_flange"] = self.physical_pose(result["pose"]).tolist()
            result["T_base_camera_mount"] = camera_mount_fk(result["joints"]).tolist()
        return result

    def physical_pose(self, raw_pose):
        require(self.T_flange_controller is not None, "Controller reference has not been verified")
        return pose_matrix(raw_pose) @ inv(self.T_flange_controller)

    def bind_reference(self, planner, state):
        # Pure kinematic queries at hypothetical joints; none of these queries moves
        # the robot. Check a fixed reference offset over multiple joint orientations.
        start = np.asarray(state["joints"], float)
        proposals = [start, np.asarray(self.cfg["observation"]["joints"][:6], float)]
        for axis in range(6):
            for direction in (-1, 1):
                q = start.copy()
                q[axis] += direction*8
                proposals.append(q)
        offsets = []
        for q in proposals:
            check_stop()
            if np.any(q < planner.low) or np.any(q > planner.high) or self.range_code(q) != 0:
                continue
            raw = self.raw_joint_pose(q)
            offsets.append(inv(planner.fk(q)) @ raw)
        require(len(offsets) >= 7, "Insufficient valid kinematic queries to verify reference origin")
        mean = mean_transform(offsets)
        max_mm = max(np.linalg.norm(h[:3, 3]-mean[:3, 3]) for h in offsets)
        max_deg = max(rotation_error(h, mean) for h in offsets)
        require(max_mm <= .5 and max_deg <= .3,
                "Controller/RoboDK kinematics disagree; difference is not a fixed reference offset")
        p = mean[:3, 3]
        require(np.linalg.norm(p[:2]) <= 1 and
                min(abs(p[2]), abs(p[2]-60)) <= 1 and
                rotation_error(mean, np.eye(4)) <= .3,
                "Reference offset differs from the supported physical flange or stock 60 mm fingertip reference; stopped")
        raw_now = self.raw_joint_pose(start)
        require(np.linalg.norm(raw_now[:3, 3]-pose_matrix(state["pose"])[:3, 3]) <= .5 and
                rotation_error(raw_now, pose_matrix(state["pose"])) <= .3,
                "Controller J2P does not match current Tool 0 / Work 0 readings")
        self.T_flange_controller = rigid(mean)
        self.reference_report = {"T_flange_controller_reference": mean.tolist(),
                                 "kinematic_query_count": len(offsets),
                                 "max_consistency_mm": float(max_mm), "max_consistency_deg": float(max_deg),
                                 "reference": "physical flange" if abs(p[2]) < 1 else "stock fingertip reference"}
        log("Verified controller reference: " + json.dumps(self.reference_report))

    def range_code(self, q):
        q = array(q, (6,), "joints")
        value = self.call("OutRange", ["J(" + ",".join("%.8f" % v for v in q) + ")", 0, 0])
        require(isinstance(value, (int, float)) and np.isfinite(value) and int(value) == value,
                "Unexpected controller OutRange response: " + str(value))
        return int(value)

    def range_code_cached(self, q):
        """Preflight memo of OutRange for an identical joint vector.

        OutRange depends only on the joints and on the controller limit
        configuration, which this program never changes. stable(), take(),
        check_start_range() and every move_joint() keep calling the live
        range_code(), so the check immediately before each motion is always
        fresh; only the repeated planning screen reuses an identical answer.
        """
        key = tuple(float(v) for v in np.asarray(q, float).reshape(-1))
        if key not in self.range_cache:
            self.range_cache[key] = self.range_code(q)
        return self.range_cache[key]

    def joint_pose_cached(self, q):
        """Planning memo of the pure J2P query for an identical joint vector.

        J2P depends only on the joint vector and the controller frame
        definitions. move_joint() and stable() keep checking real state and
        frames live before every motion; only repeated planning queries reuse
        an identical answer.
        """
        key = tuple(float(v) for v in np.asarray(q, float).reshape(-1))
        if key not in self.pose_cache:
            self.pose_cache[key] = self.joint_pose(q)
        return self.pose_cache[key]

    def check_start_range(self, q):
        code = self.range_code(q)
        if code != 0:
            axes = ", ".join("J%d=%.3f deg" % (i+1, q[i]) for i in range(6)
                             if 0 < code < 64 and code & (1 << i))
            raise ValueError("Controller rejects starting joints: OutRange=%s %s. "
                             "No documented automatic inward-recovery capability has been "
                             "verified for this controller/firmware. A valid destination "
                             "alone does not authorize leaving an invalid start. Use the "
                             "normal DENSO controller recovery procedure for the flagged "
                             "axis, then rerun. This is recovery, not teaching VIEW or a grasp. "
                             "No limits, faults or CALSET have been changed." % (code, axes))

    def raw_joint_pose(self, q):
        p = self.call("J2P", "J(" + ",".join("%.8f" % v for v in q) + ")")
        require(len(p) >= 6, "Unexpected J2P result")
        return pose_matrix(p)

    def joint_pose(self, q):
        require(self.T_flange_controller is not None, "Controller reference unverified")
        return self.raw_joint_pose(q) @ inv(self.T_flange_controller)

    def stable(self, powered=False, empty=False):
        self.check_frames()
        a = self.state()
        pause(0.20)
        b = self.state()
        require(not a["busy"] and not b["busy"] and not a["hand_busy"] and not b["hand_busy"] and
                np.max(np.abs(np.asarray(a["joints"])-b["joints"])) < 0.05,
                "Robot/gripper must be stationary")
        require(not a["error"] and not b["error"],
                "Controller error: " + str(a["error_description"] if a["error"] else b["error_description"]))
        if powered:
            require(a["servo"] and b["servo"],
                    "Motor power is OFF or changing (@SERVO_ON). Stopped; no automatic "
                    "re-enable or route resume. Check DENSO status before starting a new run.")
        if empty:
            require(not b["hand_holding"], "Remove the held object using normal controls first")
        self.check_start_range(b["joints"])
        return b

    def prepare_for_motion(self, folder, empty=False):
        """Startup only: confirm motor-on, request it once, then verify readiness.

        The operator runs this code in RoboDK. Development never invokes it.
        Motor loss later remains a stop; no automatic re-enable or route replay.
        """
        require(not self.owned, "Motor preparation is allowed only before arm acquisition")
        readings = []
        path = Path(folder)/"motor_readiness.json"

        def record(stage, state=None, error=None):
            entry = {"time_utc": stamp(), "stage": stage}
            if state is not None:
                entry.update(state=state, frames=self.frames(), external_overrides={
                    k: float(self.var(k)) for k in SPEED_VARS[:3]})
            if error is not None:
                entry["error"] = {"type": type(error).__name__, "message": str(error)}
            readings.append(entry)
            write_json(path, {"program_revision": PROGRAM_REVISION,
                             "motor_status_source": "robot @SERVO_ON (DENSO VT_BOOL)",
                             "readings": readings})

        try:
            check_stop()
            state = self.stable(empty=empty)
            record("initial", state)
            # Startup gate BEFORE any confirmation, motor-on request, gripper
            # action or motion. motor_readiness.json above already records the
            # fresh override readings that decide it.
            self.check_external_overrides()
            if state["servo"]:
                state = self.stable(powered=True, empty=empty)
                record("already_on_verified", state)
                return state
            confirm("ENABLE ARM MOTOR POWER? The controller reports @SERVO_ON=false.\n\n"
                    "OK authorizes one DENSO motor-on request and verification of @SERVO_ON. "
                    "Keep the robot area clear and the physical stop accessible. "
                    "Cancel ends the run without requesting motor-on.\n\n"
                    "This step enables power only. Arm movements still require the separate "
                    "calibration/pick confirmation. Command speed/acceleration/deceleration "
                    "remain 100%; other live settings and faults are left unchanged.")
            check_stop()
            state = self.stable(empty=empty)
            record("startup_confirmed", state)
            if not state["servo"]:
                # DENSO sequence: acquire authority first, retaining all current
                # settings (Keep=1). The later motion confirmation still gates moves.
                self._acquire_arm()
                check_stop()
                record("motor_on_requesting")
                # Motor(State=1, NoWait=1), RC8 provider guide 5.2.29.44.
                # Poll below instead of blocking beyond the transport timeout.
                self.call("Motor", [1, 1])
                record("motor_on_request_accepted")
                end = time.monotonic()+10.0
                while time.monotonic() < end:
                    check_stop()
                    state = self.stable(empty=empty)
                    record("verifying_motor_on", state)
                    if state["servo"]:
                        break
                    pause(0.10)
                else:
                    raise TimeoutError("Motor-on did not verify @SERVO_ON=true within 10 seconds. "
                                       "No automatic retry. Check DENSO status; no fault reset "
                                       "or other setting change was requested.")
            state = self.stable(powered=True, empty=empty)
            record("motor_on_verified", state)
            log("Motor power verified ON (@SERVO_ON); motion confirmation is still required.")
            return state
        except BaseException as exc:
            record("stopped", error=exc)
            raise

    def check_external_overrides(self):
        """Refuse to start with a pendant override below EXTERNAL_OVERRIDE_FLOOR.

        Readings are fresh and read-only; no external override is ever written.
        Raises before any motor-on request, confirmation, gripper action or
        motion, and keeps the fresh values as this run's change-detection baseline.
        """
        fresh = [float(self.var(k)) for k in SPEED_VARS[:3]]
        self.initial_external = fresh
        if min(fresh) < EXTERNAL_OVERRIDE_FLOOR:
            raise ValueError(
                "External pendant override is below the %g%% startup floor: %s. "
                "At these multipliers the arm crawls, and any pendant change during "
                "the run would stop it. Set the pendant speed/acceleration override "
                "to 100%% and restart; no motion has been requested and this program "
                "never writes these values." % (
                    EXTERNAL_OVERRIDE_FLOOR,
                    ", ".join("%s=%.6g" % (k, v) for k, v in zip(SPEED_VARS[:3], fresh))))
        return fresh

    def external_override_note(self):
        """One confirmation line when the pendant limits effective motion."""
        values = [float(v) for v in (self.initial_external or [])]
        if len(values) != 3 or min(values) >= 100.0:
            return ""
        # No trailing period: callers place this sentence before a literal that
        # already starts with ". ".
        return ("Effective speed/acceleration/deceleration are limited by the pendant "
                "override (%s percent); changing the pendant override during the run "
                "will stop the run" % ", ".join(
                    "%s=%.6g" % (k, v) for k, v in zip(SPEED_VARS[:3], values)))

    def speed_report(self):
        report = {k: float(self.var(k)) for k in SPEED_VARS}
        report["requested_config"] = self.setup["motion"]
        report["legacy_speed_percent"] = self.cfg["speed_percent"]
        report["policy"] = "Command-local 100% S/A/D; external overrides unchanged. No ExtSpeed writes."
        log(json.dumps(report, indent=2))
        return report

    def take(self):
        self.stable(powered=True)
        self._acquire_arm()

    def _acquire_arm(self):
        self.check_frames()
        require(all(0 < x <= 100 for x in self.initial_external), "Invalid external speed settings")
        require(np.allclose([self.var(k) for k in SPEED_VARS[:3]], self.initial_external,
                            atol=1e-6, rtol=0), "External speed/acceleration override changed")
        if not self.owned:
            # Keep=1 preserves internal speed and Tool/Work selections. Every
            # move still supplies explicit SPEED/ACCEL/DECEL=100 command options.
            self.call("TakeArm", [0, 1])
            self.owned = True
            self.check_frames()

    def grip_ok(self):
        return (bool(self.hand("HandHoldState")) and
                abs(float(self.hand("HandCurPos"))-self.setup["cube_width_mm"]) <=
                self.setup["gripper"]["grip_width_tolerance_mm"])

    def check_motion_state(self):
        check_stop()
        require(self.var("@SERVO_ON"), "Motor power lost")
        require(not self.var("@ERROR_CODE"), "Controller error: " + str(self.var("@ERROR_DESCRIPTION")))
        require(self.var("@CURRENT_TOOL") == 0 and self.var("@CURRENT_WORK") == 0,
                "Tool/Work selection changed")
        if self.holding_required:
            require(self.grip_ok(), "Grasp confirmation lost; stopping with no release")

    def move_joint(self, q0, q1):
        require(self.owned, "Arm has not been acquired")
        command_started = time.perf_counter()
        self.check_frames()
        self.check_motion_state()
        actual = np.asarray(self.var("@CURRENT_ANGLE"))[:6]
        require(np.max(np.abs(actual-q0)) <= 0.25, "Robot moved since planning; discard route")
        require(self.range_code(q1) == 0, "Controller now rejects target joints")
        # Exact-equality change detection is unchanged; only the failure text now
        # names the axis that moved. This program never writes these values.
        fresh = [float(self.var(k)) for k in SPEED_VARS[:3]]
        if not np.allclose(fresh, self.initial_external, atol=1e-6, rtol=0):
            raise ValueError(
                "External speed/acceleration override changed: " + ", ".join(
                    "%s %.6g -> %.6g" % (k, old, new)
                    for k, old, new in zip(SPEED_VARS[:3], self.initial_external, fresh)) +
                ". The arm did not move for this step: the check runs before the motion "
                "command, and this program never writes these values (it only reads them). "
                "Restore the pendant override and restart the run.")
        m = self.setup["motion"]
        opts = "SPEED=%.6g,ACCEL=%.6g,DECEL=%.6g,NEXT" % (
            m["speed_percent"], m["acceleration_percent"], m["deceleration_percent"])
        # PTP joint target, end-position stop; no blending. NEXT allows polling/halt.
        self.client.robot_move(self.arm, 1, [list(map(float, q1)), "J", "@E"], opts)
        TIMINGS.add("controller.move_joint.command_issue", time.perf_counter()-command_started)
        begin = time.monotonic()
        polling_started = time.perf_counter()
        try:
            while time.monotonic()-begin < 120:
                self.check_motion_state()
                q = np.asarray(self.var("@CURRENT_ANGLE"))[:6]
                busy = self.var("@BUSY_STATUS")
                require(np.all(q >= np.minimum(q0, q1)-0.5) and
                        np.all(q <= np.maximum(q0, q1)+0.5), "Joint motion left planned interval")
                if not busy and np.max(np.abs(q-q1)) <= 0.20:
                    return
                if not busy and time.monotonic()-begin > 2:
                    raise RuntimeError("Controller stopped before reaching waypoint")
                pause(0.05)
            raise TimeoutError("Motion timed out")
        finally:
            TIMINGS.add("controller.move_joint.polling", time.perf_counter()-polling_started)

    def wait_hand(self, predicate):
        end = time.monotonic()+10
        while time.monotonic() < end:
            self.check_motion_state()
            if not self.hand("HandBusyState") and predicate():
                return
            pause(0.05)
        raise TimeoutError("Gripper timed out; check object, position and hold status")

    def halt(self):
        if self.arm is not None:
            try:
                self.client.robot_halt(self.arm, "")
            except Exception as exc:
                log("HALT NOT CONFIRMED; use physical stop. " + str(exc))
            try:
                self.hand("HandStop")
            except Exception as exc:
                log("HandStop not confirmed: " + str(exc))

    def __exit__(self, typ, value, tb):
        if typ is not None and self.owned:
            self.halt()
        actions = []
        if self.owned:
            actions.append(lambda: self.call("GiveArm"))
        for handle in self.variable_handles.values():
            actions.append(lambda handle=handle: self.client.variable_release(handle))
        self.variable_handles.clear()
        if self.arm is not None:
            actions.append(lambda: self.client.robot_release(self.arm))
        if self.ctrl is not None:
            actions.append(lambda: self.client.controller_disconnect(self.ctrl))
        if self.started:
            actions.append(self.client.service_stop)
        for action in actions:
            try:
                action()
            except Exception as exc:
                log("Connection cleanup: " + str(exc))
        # No automatic motor off, fault reset, speed changes or object release.


class Camera:
    def __init__(self, vision, folder, *, snapshot_timeout=3):
        self.vision, self.folder = vision, Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.board, self.dictionary = get_board()
        password = str(vision.get("camera_password", ""))
        require(password and password != "CAMERA_PASSWORD",
                "Set camera_password in " + str(CONFIG_FILE) + " (see guide.md).")
        self.http = HTTPCamera(vision["camera_ip"], vision["camera_username"],
                               password, timeout=snapshot_timeout)

    def frame(self, label, controller=None, *, before=None):
        if controller and before is None:
            with TIMINGS.phase("camera.frame.stable_before"):
                before = controller.stable()
        record = {"source": "fresh HTTP request; sensor timestamp unavailable", "started_utc": stamp(),
                  "label": label, "state_before": before}
        prefix = self.folder / (record["started_utc"] + "_" + label)
        frame = None
        try:
            with TIMINGS.phase("camera.frame.http_capture"):
                try:
                    data = self.http.capture()
                except TimeoutError as exc:
                    raise TimeoutError(f"Camera snapshot {label} timed out "
                                       f"({self.http.timeout:g} s socket timeout)") from exc
            prefix.with_suffix(".jpg").write_bytes(data)
            with TIMINGS.phase("camera.frame.decode"):
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                require(frame is not None, "JPEG decoding failed")
                require(frame.shape[:2] == (1080, 1920), "Camera resolution changed")
            record["image_sha256"] = hashlib.sha256(data).hexdigest()
            record["completed_utc"] = stamp()
            record["image_file"] = prefix.with_suffix(".jpg").name
            if controller:
                with TIMINGS.phase("camera.frame.stable_after"):
                    after = controller.stable()
                require(np.max(np.abs(np.asarray(before["joints"])-after["joints"])) < 0.05,
                        "Robot moved during exposure")
                record["state_after"] = after
                record["T_base_flange"] = after["T_base_flange"]
                record["T_base_camera_mount"] = after["T_base_camera_mount"]
            show_image(frame, "FRESH HTTP SNAPSHOT " + record["completed_utc"] + " | " + label)
            write_json(prefix.with_suffix(".json"), record)
            return frame, record, prefix
        except BaseException as exc:
            record["error"] = str(exc)
            write_json(prefix.with_suffix(".json"), record)
            canvas = frame if frame is not None else np.zeros((1080, 1920, 3), np.uint8)
            # A failed request displays a new error panel, never yesterday's image as live.
            try:
                show_image(canvas, "CAPTURE FAILED (not live): " + str(exc))
            except Exception:
                pass  # A stop/window error must not hide the actual capture failure.
            raise

    def observation(self, controller, label, role, before_state=None):
        # Discard one request after settling: cache=off is not a sensor timestamp.
        pause(0.7)
        # The caller may supply the stationary state a move already verified, so
        # this settle frame does not repeat the same controller.stable() call.
        # frame() keeps every existing check, including "Robot moved during exposure".
        _, settled, _ = self.frame(label + "_settle", controller, before=before_state)
        # Consecutive snapshots share their stationary boundary reading. Both
        # requests and the pose checks at the start, boundary and end remain.
        check_stop()
        frame, rec, prefix = self.frame(label, controller, before=settled["state_after"])
        rec["role"] = role
        detection_started = time.perf_counter()
        markers, ids, rejected = cv2.aruco.detectMarkers(frame, self.dictionary)
        annotated = frame.copy()
        # Ignore dictionary markers outside this exact printed board.
        known_ids = set(self.board.ids.reshape(-1).astype(int).tolist())
        if ids is not None:
            keep = [i for i, marker_id in enumerate(ids.reshape(-1)) if int(marker_id) in known_ids]
            markers = [markers[i] for i in keep]
            ids = ids[keep] if keep else None
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(annotated, markers, ids)
        corners, corner_ids = None, None
        if ids is not None and len(ids) >= 3:
            # OpenCV still refines the actual chess corner with cornerSubPix.
            # One visible adjacent marker supports partial views near a sensor
            # edge; requiring both prevents the necessary image coverage here.
            _, corners, corner_ids = cv2.aruco.interpolateCornersCharuco(
                markers, ids, frame, self.board, minMarkers=1)
            if corner_ids is not None:
                xy = corners.reshape(-1, 2)
                keep = np.all((xy >= [12, 12]) & (xy <= [1907, 1067]), axis=1)
                corners, corner_ids = corners[keep], corner_ids[keep]
        count = 0 if corner_ids is None else len(corner_ids)
        area, rows, columns = 0.0, 0, 0
        if count:
            # Show and retain partial detections even when they cannot calibrate.
            # The old code hid them behind the >=12-corner acceptance condition.
            cv2.aruco.drawDetectedCornersCharuco(annotated, corners, corner_ids)
            rec["corners_px"] = corners.reshape(-1, 2).tolist()
            rec["corner_ids"] = corner_ids.reshape(-1).astype(int).tolist()
            obj = np.asarray(self.board.chessboardCorners)[corner_ids.reshape(-1)]
            rows, columns = len(np.unique(obj[:, 1])), len(np.unique(obj[:, 0]))
            hull = cv2.convexHull(corners.astype(np.float32))
            area = float(cv2.contourArea(hull))
        marker_count = 0 if ids is None else len(ids)
        rec["detection"] = {"marker_count": marker_count,
                            "marker_ids": [] if ids is None else ids.reshape(-1).astype(int).tolist(),
                            "charuco_corner_count": count, "board_rows": rows, "board_columns": columns,
                            "min_adjacent_markers": 1, "corner_sensor_inset_px": 12,
                            "corner_hull_area_px2": area,
                            "requirements": {"corners": 12, "hull_area_px2": 8000,
                                             "min_rows": 2, "min_columns": 2}}
        # This homography is a provisional framing measurement, never an accepted
        # camera calibration. Image-directed proposals also need a separately
        # measured robot/image response and the complete physical route planner.
        centre = np.array([frame.shape[1]/2, frame.shape[0]/2])
        cv2.drawMarker(annotated, tuple(centre.astype(int)), (255, 255, 255),
                       cv2.MARKER_CROSS, 45, 2)
        if marker_count >= 3:
            obj_by_id = {int(i): np.asarray(p)[:, :2] for i, p in
                         zip(self.board.ids.reshape(-1), self.board.objPoints)}
            src = np.concatenate([obj_by_id[int(i)] for i in ids.reshape(-1)]).astype(np.float32)
            dst = np.concatenate(markers).reshape(-1, 2).astype(np.float32)
            h, _ = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
            if h is not None:
                w, height = BOARD["width_mm"], BOARD["height_mm"]
                outline = np.array([[0, 0], [w, 0], [w, height], [0, height],
                                    [w/2, height/2]], np.float32)
                projected = cv2.perspectiveTransform(outline.reshape(-1, 1, 2), h).reshape(-1, 2)
                if np.isfinite(projected).all() and np.max(np.abs(projected)) < 100000:
                    rec["framing_guide"] = {"pattern_outline_px": projected[:4].tolist(),
                                            "pattern_centre_px": projected[4].tolist(),
                                            "image_centre_px": centre.tolist()}
                    rec["framing_guide"]["method"] = "provisional marker homography; not accepted calibration"
                    # The user's saved image retained a blue object on the board.
                    # Do not compensate for that obstruction by moving the robot.
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    blue = cv2.inRange(hsv, np.asarray(self.vision["hsv_lower"], np.uint8),
                                       np.asarray(self.vision["hsv_upper"], np.uint8))
                    pattern_mask = np.zeros(frame.shape[:2], np.uint8)
                    cv2.fillConvexPoly(pattern_mask, np.rint(projected[:4]).astype(np.int32), 255)
                    blue = cv2.bitwise_and(blue, pattern_mask)
                    blobs, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    blue_area = max((cv2.contourArea(c) for c in blobs), default=0.0)
                    rec["blue_region_on_pattern_px2"] = float(blue_area)
                    cv2.polylines(annotated, [np.rint(projected[:4]).astype(np.int32)],
                                  True, (0, 255, 255), 2)
                    start = np.clip(np.rint(projected[4]), [0, 66],
                                    [frame.shape[1]-1, frame.shape[0]-1]).astype(int)
                    if np.linalg.norm(projected[4]-centre) > 60:
                        cv2.arrowedLine(annotated, tuple(start), tuple(centre.astype(int)),
                                        (0, 255, 255), 3, tipLength=0.08)
        valid = (count >= 12 and area >= 8000 and rows >= 2 and columns >= 2 and
                 rec.get("blue_region_on_pattern_px2", 0) < 400)
        rec["accepted"] = bool(valid)
        rec["reason"] = "%d markers; %d/12 ChArUco corners, %d rows x %d columns, hull %.0f px^2" % (
            marker_count, count, rows, columns, area)
        if not valid:
            rec["reason"] += "; need >=12 corners in >=2 rows/columns over >=8000 px^2"
        if rec.get("blue_region_on_pattern_px2", 0) >= 400:
            rec["reason"] += "; blue region obstructs the pattern: remove the object before motion"
        TIMINGS.add("camera.observation.detection", time.perf_counter()-detection_started)
        shown = show_image(annotated, "FRESH SNAPSHOT | " + label + " | " + rec["reason"])
        writes_started = time.perf_counter()
        cv2.imwrite(str(prefix.with_name(prefix.name+"_corners.png")), shown)
        write_json(prefix.with_suffix(".json"), rec)
        TIMINGS.add("camera.observation.writes", time.perf_counter()-writes_started)
        return rec

    def close(self):
        self.http.close()


def box_vertices(bounds):
    lo, hi = array(bounds, (2, 3), "box")
    require(np.all(hi > lo), "Invalid box bounds")
    return np.array(list(itertools.product(*zip(lo, hi))))


def box_mesh(bounds):
    p = box_vertices(bounds)
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
             (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    triangles = [p[i] for a, b, c, d in faces for i in (a, b, c, a, c, d)]
    return robomath.Mat(np.asarray(triangles).T.tolist())


class Planner:
    """RoboDK geometry + joint-space RRT; no RoboDK robot Move/driver calls.

    Generated objects are positioned explicitly at every sampled state, so collision
    checking does not depend on RoboDK's active Tool 1 or its TCP definition.
    """
    def __init__(self, rdk, controller, setup, table=None, workspace=None, grasp_tcp=None):
        self.rdk, self.controller, self.setup = rdk, controller, setup
        # Logical roles (by item name, never by RoboDK object identity) and
        # explicit pair states let scene_signature() prove that a rebuilt
        # collision scene is identical before reusing a preflight. They must
        # exist before the robot role and before create_geometry().
        self.role_by_name = {}
        self.pair_state = {}
        self.preflighted = {}
        # Diagnostics for hard positions: why a path was rejected, how far the
        # route search got, and which fallback connected. Written by run_pick.
        self.planning_notes = []
        self.last_rrt_candidates = 0
        if rdk.RunMode() != robolink.RUNMODE_SIMULATE:
            rdk.setRunMode(robolink.RUNMODE_SIMULATE)
        # Rendering is disabled while planning: collision checking uses item poses,
        # not the 3D view, and redrawing every sample only slows the checks and
        # makes the simulated arm jump through random RRT configurations.
        # close() restores rendering. Nothing about the checks changes.
        rdk.Render(False)
        log("RoboDK is used for simulated planning (rendering off while planning). "
            "Physical calibrate/pick motion uses b-CAP after confirmation.")
        self.robot = rdk.Item(setup["robot"], robolink.ITEM_TYPE_ROBOT)
        require(self.robot.Valid(), "Robot not found: " + setup["robot"])
        self.role_by_name[str(self.robot.Name())] = "robot"
        log("Planning station: " + rdk.ActiveStation().Name())
        require(len(rdk.ItemList(robolink.ITEM_TYPE_ROBOT)) == 1,
                "Use a station containing only the intended COBOTTA robot")
        self.base_world = np.asarray(self.robot.PoseAbs().rows)
        low, high, _ = self.robot.JointLimits()
        # Published nominal limits narrow the search, never widen station/controller limits.
        self.low = np.maximum(np.asarray(low.list())[:6], [-150, -60, 18, -170, -95, -170])
        self.high = np.minimum(np.asarray(high.list())[:6], [150, 100, 140, 170, 135, 170])
        self.items, self.moving = [], []
        self.camera_item = None
        self.cache = {}
        self.rejection_reason = ""
        self.held_box = None
        self.target_cube = None
        self.target_pose = None
        self.table = table
        self.workspace = workspace
        self.grasp_tcp = rigid(grasp_tcp) if grasp_tcp is not None else None
        self.rng = np.random.default_rng(47833)
        self.margin = float(setup["planning"]["margin_mm"])
        self.step = float(setup["planning"]["joint_sample_deg"])
        require(0 < self.step <= 0.5 and self.margin >= 2, "Invalid collision sampling/margin")
        require(np.all(self.low < self.high), "Robot model joint limits are invalid")
        state = controller.stable()
        self.T_model_physical = np.eye(4)
        self.bind_model_reference(state)
        controller.bind_reference(self, state)
        self.calibration_anchor_xy = None
        if self.workspace is None:
            # User's paper extends 70 mm left, 140 right, 40 behind, 240 forward
            # from VIEW's gripper projection. A 35 mm disk (40 minus 5 margin)
            # fits that rectangle without guessing the operator-to-base axis signs.
            self.grasp_tcp = grasp_transform(setup)
            saved_q = controller.cfg["observation"]["joints"][:6]
            self.calibration_anchor_xy = (self.fk(saved_q) @ self.grasp_tcp)[:2, 3]
        self.sync_check(state)
        self.purge_stale_generated_items()
        try:
            self.create_geometry()
        except BaseException:
            self.close()
            raise
        self.rdk.setCollisionActive(robolink.COLLISION_ON)

    def fk(self, q):
        return np.asarray(self.robot.SolveFK(list(map(float, q))).rows) @ self.T_model_physical

    def bind_model_reference(self, state):
        start = np.asarray(state["joints"], float)
        probes = [start]
        for i in range(6):
            for sign in (-1, 1):
                q = start.copy()
                q[i] += 8*sign
                if np.all(q >= self.low) and np.all(q <= self.high):
                    probes.append(q)
        offsets = [inv(documented_physical_fk(q)) @
                   np.asarray(self.robot.SolveFK(q.tolist()).rows) for q in probes]
        mean = mean_transform(offsets)
        p = mean[:3, 3]
        require(len(offsets) >= 7 and
                max(np.linalg.norm(h[:3, 3]-p) for h in offsets) <= .8 and
                max(rotation_error(h, mean) for h in offsets) <= .3 and
                np.linalg.norm(p[:2]) <= 1 and min(abs(p[2]), abs(p[2]-60)) <= 1 and
                rotation_error(mean, np.eye(4)) <= .3,
                "RoboDK model does not match the documented stock COBOTTA kinematics "
                "through a supported flange/fingertip reference; stopped")
        self.T_model_physical = inv(mean)
        log("Verified RoboDK model reference offset from physical mounting flange (mm): " + str(p.tolist()))

    def sync_check(self, state):
        q = array(state["joints"], (6,), "controller joints")
        actual = self.controller.physical_pose(state["pose"])
        model = self.fk(q)
        require(np.linalg.norm(actual[:3, 3]-model[:3, 3]) <=
                self.setup["planning"]["position_tolerance_mm"] and
                rotation_error(actual, model) <= self.setup["planning"]["angle_tolerance_deg"],
                "RoboDK FK at REAL controller joints disagrees with Tool 0 / Work 0. "
                "Correct the robot model/calibration/base convention; no joint offsets were guessed.")
        self.robot.setJoints(q.tolist())

    def make_box(self, name, bounds, world_pose):
        item = self.rdk.AddShape(box_mesh(bounds))
        require(item.Valid(), "RoboDK could not create collision geometry")
        item.setName("AUTO_" + name + "_" + stamp())
        item.setPoseAbs(robomath.Mat(world_pose.tolist()))
        item.setColor([0.9, 0.45, 0.15, 0.35])
        self.items.append(item)
        self.role_by_name[str(item.Name())] = "box:" + name
        return item

    def enable_pair(self, a, b, ia=0, ib=0, on=True):
        require(self.rdk.setCollisionActivePair(robolink.COLLISION_ON if on else robolink.COLLISION_OFF,
                                              a, b, ia, ib), "Could not configure collision pair")
        self.pair_state[(self._role(a), int(ia), self._role(b), int(ib))] = bool(on)

    def purge_stale_generated_items(self):
        # A killed RoboDK session can leave AUTO_* collision items behind. They
        # would otherwise be added as obstacles below and make every pose collide.
        stale = []
        for item in self.rdk.ItemList(robolink.ITEM_TYPE_OBJECT):
            try:
                if str(item.Name()).startswith("AUTO_"):
                    stale.append(item)
            except Exception:
                continue
        for item in stale:
            try:
                item.Delete()
            except Exception as exc:
                log("Could not delete stale generated item: " + str(exc))
        if stale:
            log("Removed %d stale AUTO_* collision item(s) left by a previous run." % len(stale))

    def create_geometry(self):
        if self.table:
            n = np.asarray(self.table["normal_base"])
            p = np.asarray(self.table["point_base_mm"])
            x = np.cross([0, 1, 0], n)
            x /= np.linalg.norm(x)
            table_h = transform(np.column_stack([x, np.cross(n, x), n]), p)
            top_margin = self.margin
        else:
            table_h = transform(t=[0, 0, self.setup["table_initial_z_mm"]])
            top_margin = self.setup["table_initial_uncertainty_mm"] + self.margin
        self.table_h = table_h
        table = self.make_box("table", [[-1000, -1000, -1000], [1000, 1000, top_margin]],
                              self.base_world @ table_h)
        obstacles = [table]
        for i, bounds in enumerate(self.setup["extra_obstacles_base_aabbs_mm"]):
            a = np.asarray(bounds) + np.array([[-self.margin]*3, [self.margin]*3])
            obstacles.append(self.make_box("obstacle%d" % i, a, self.base_world))
        self.obstacles = obstacles
        # The old body AABB filled the rounded CAD corners and falsely met the
        # camera near J6=6.2 degrees. This closed convex CAD envelope retains
        # every stock-body surface plus 2 mm; fingers keep their opening sweep.
        body = self.rdk.AddFile(str(ROOT/GRIPPER_BODY_ENVELOPE), self.rdk.ActiveStation())
        require(body.Valid(), "RoboDK could not load stock gripper collision geometry")
        self.items.append(body)
        body.setName("AUTO_gripper_body_" + stamp())
        body.setColor([0.9, 0.45, 0.15, 0.35])
        body.setPoseAbs(robomath.Mat((self.base_world @ self.fk(self.robot.Joints().list()[:6])).tolist()))
        self.role_by_name[str(body.Name())] = "gripper_body"
        self.moving.append((body, None))
        bounds = [([[-16, -29, 36], [16, 29, 60]], "finger_swept_envelope")]
        for box, name in bounds:
            enlarged = np.asarray(box) + np.array([[-self.margin]*3, [self.margin]*3])
            item = self.make_box(name, enlarged, self.base_world @ self.fk(self.robot.Joints().list()[:6]))
            self.moving.append((item, enlarged))
        # This asset contains the factory camera, bracket, fasteners and routed
        # cable, with the wrist casing removed. It encloses both documented
        # factory mounting positions with a 2 mm margin, in physical J5 axes.
        # Source triangles are subdivided to <=4mm axis spans before grouping,
        # removing false corner fill from the older long-triangle boxes.
        camera = self.rdk.AddFile(str(ROOT/CAMERA_ENVELOPE), self.rdk.ActiveStation())
        require(camera.Valid(), "RoboDK could not load factory camera collision geometry")
        self.items.append(camera)
        self.camera_item = camera
        camera.setName("AUTO_factory_camera_J5_" + stamp())
        camera.setColor([0.9, 0.45, 0.15, 0.35])
        camera.setPoseAbs(robomath.Mat((self.base_world @
                          camera_mount_fk(self.robot.Joints().list()[:6])).tolist()))
        self.role_by_name[str(camera.Name())] = "factory_camera"
        self.moving.append((camera, None))
        # Explicit pair setup: the table's intentional contact with the stationary base
        # is excluded. Gripper mounting interfaces are links 5/6; the camera's
        # only mounting interface is link 5. Camera/link-4 checking stays enabled.
        for obstacle in obstacles:
            for link in range(1, 7):
                self.enable_pair(self.robot, obstacle, link)
            self.enable_pair(self.robot, obstacle, 0, on=False)
        for item, _ in self.moving:
            for link in range(7):
                mounting_contact = link == 5 if item == camera else link in (5, 6)
                self.enable_pair(self.robot, item, link, on=not mounting_contact)
            for obstacle in obstacles:
                self.enable_pair(item, obstacle)
            for other, _ in self.moving:
                if item != other:
                    # The stock gripper body and the factory camera envelope are
                    # rigid wrist parts whose relative pose depends only on J6.
                    # The envelope deliberately covers two factory profiles plus
                    # the CAD cable loop with a 2 mm margin. The operator verified
                    # the real wrist rotates freely in both directions with a cube
                    # held and asked for that one pair to be removed, so it is
                    # disabled. Every other camera pair (finger sweep, links,
                    # table, payload, target cube, station objects) stays enabled.
                    body_camera = ({self._role(item), self._role(other)} ==
                                   {"gripper_body", "factory_camera"})
                    self.enable_pair(item, other,
                                     on=(item == camera or other == camera) and not body_camera)
        for i in range(7):
            for j in range(i+2, 7):
                self.enable_pair(self.robot, self.robot, i, j)
        # Include existing station objects in the moving enclosure checks.
        for other in self.rdk.ItemList(robolink.ITEM_TYPE_OBJECT):
            if other not in self.items:
                for item, _ in self.moving:
                    self.enable_pair(item, other)
        log("Collision model: station arm/objects + table + stock CAD gripper body/finger sweep + "
            "refined factory camera/bracket/CAD cable envelope on J5 "
            "(subdivided CAD, 2 mm margin, both factory profiles). "
            "Camera checked against link 4, link 6 and gripper; only its J5 mounting contact excluded. "
            "Loose cable deviations, added fixtures and people are not modelled; sampling is not a safety certification.")

    def set_payload(self, bounds=None):
        if self.held_box is not None:
            item, _ = self.held_box
            self.moving.remove(self.held_box)
            self.items.remove(item)
            self._forget(item)
            item.Delete()
            self.held_box = None
        if bounds is not None:
            bounds = np.asarray(bounds) + np.array([[-self.margin]*3, [self.margin]*3])
            item = self.make_box("held_cube", bounds, self.base_world)
            self.held_box = (item, bounds)
            for link in range(0, 5):
                self.enable_pair(self.robot, item, link)
            for link in (5, 6):
                self.enable_pair(self.robot, item, link, on=False)
            for other, _ in self.moving:
                self.enable_pair(item, other, on=other == self.camera_item)
            for other in self.rdk.ItemList(robolink.ITEM_TYPE_OBJECT):
                if other not in self.items or other in self.obstacles:
                    self.enable_pair(item, other)
            self.moving.append(self.held_box)
        self.cache.clear()

    def set_target_cube(self, cube=None, cal=None):
        if self.target_cube is not None:
            self.items.remove(self.target_cube)
            self._forget(self.target_cube)
            self.target_cube.Delete()
            self.target_cube = None
            self.target_pose = None
        if cube is not None:
            n = np.asarray(cal["table"]["normal_base"])
            y = np.asarray(cube["closing_axis_base"])
            h = transform(np.column_stack([np.cross(y, n), y, n]), cube["centre_base_mm"])
            w, z = self.setup["cube_width_mm"], self.setup["cube_height_mm"]
            item = self.make_box("target_cube", [[-w/2, -w/2, -z/2], [w/2, w/2, z/2]], self.base_world @ h)
            self.target_cube = item
            self.target_pose = self.base_world @ h
            for link in range(7):
                self.enable_pair(self.robot, item, link)
            for moving, _ in self.moving:
                # The full opening-sweep finger box encloses the intended grasp gap.
                # Only that intentional target/finger contact is excluded.
                self.enable_pair(moving, item, on="finger_swept_envelope" not in moving.Name())
            # Only the supporting table is exempt. Extra fixtures stay checked.
            self.enable_pair(item, self.obstacles[0], on=False)  # target rests on table
            for obstacle in self.obstacles[1:]:
                self.enable_pair(item, obstacle)
        self.cache.clear()

    def _role(self, item):
        # Roles are keyed by item name: RoboDK Item objects are not reliable
        # dictionary keys, and a re-created object always gets a new name.
        # Station objects this program did not create are named here so that a
        # rebuilt or replaced object changes the signature instead of silently
        # reusing a check made against the old one.
        try:
            name = str(item.Name())
        except Exception:
            return "unreadable:" + str(id(item))
        return self.role_by_name.get(name, "station:" + name)

    def _forget(self, item):
        try:
            role = self.role_by_name.pop(str(item.Name()), None)
        except Exception:
            role = None
        if role is not None:
            self.pair_state = {key: on for key, on in self.pair_state.items()
                               if role not in (key[0], key[2])}

    def _path_key(self, path):
        return tuple(tuple(np.round(np.asarray(q, float).ravel(), 6).tolist()) for q in path)

    def scene_signature(self):
        """Content signature of the collision scene that valid() actually reads.

        Used only to decide whether an identical preflight may be reused: any
        difference in the moving items and their bounds, the target cube or
        held payload, the explicit collision-pair enables, the paper workspace,
        the preliminary anchor or the search limits gives a different value and
        forces a new check. Station-global collisions from unrelated station
        objects are outside this signature, exactly as they are outside the
        existing per-joint valid() cache.
        """
        def nums(values):
            a = np.asarray(values, float)
            return tuple(np.round(a.ravel(), 6).tolist()) if a.size else ()
        parts = [
            ("moving", tuple(sorted(
                (self._role(item), ("mesh",) if bounds is None else nums(bounds))
                for item, bounds in self.moving))),
            ("target", None if self.target_cube is None else
             (self._role(self.target_cube), nums(self.target_pose))),
            ("payload", None if self.held_box is None else
             (self._role(self.held_box[0]), nums(self.held_box[1]))),
            ("obstacles", tuple(sorted(self._role(item) for item in self.obstacles))),
            ("pairs", tuple(sorted((a, ia, b, ib, on) for (a, ia, b, ib), on
                                   in self.pair_state.items()))),
            ("workspace", None if self.workspace is None else hashlib.sha256(
                json.dumps(self.workspace, sort_keys=True, default=str).encode()).hexdigest()),
            ("anchor", None if self.calibration_anchor_xy is None else
             nums(self.calibration_anchor_xy)),
            ("limits", nums(self.low), nums(self.high), float(self.margin), float(self.step)),
            ("base_world", nums(self.base_world)),
        ]
        return hashlib.sha256(repr(parts).encode()).hexdigest()

    def valid(self, q):
        check_stop()
        self.rejection_reason = ""
        q = np.asarray(q)
        if np.any(q < self.low) or np.any(q > self.high) or not np.isfinite(q).all():
            self.rejection_reason = "Joint values outside model/published limits: " + str(q.tolist())
            return False
        key = tuple(np.round(q, 6))
        if key in self.cache:
            valid, self.rejection_reason = self.cache[key]
            return valid
        self.robot.setJoints(q.tolist())
        flange = self.fk(q)
        if self.calibration_anchor_xy is not None:
            g = (flange @ self.grasp_tcp)[:3, 3]
            if np.linalg.norm(g[:2]-self.calibration_anchor_xy) > 35.0:
                self.rejection_reason = (
                    "Grasp-centre projection is %.1f mm from the saved VIEW projection; "
                    "the preliminary paper region permits 35 mm. If paper measurements "
                    "refer to this current pose, use set-view to preview and save it "
                    "without motion. No collision has been inferred from this boundary check."
                    % np.linalg.norm(g[:2]-self.calibration_anchor_xy))
                self.cache[key] = (False, self.rejection_reason)
                return False
        if self.workspace is not None:
            g = (flange @ self.grasp_tcp)[:3, 3]
            coords = np.asarray(self.workspace["axes_base"]).T @ (g-np.asarray(self.workspace["origin_base_mm"]))
            poly = np.asarray(self.workspace["polygon_table_xy_mm"], np.float32)
            if cv2.pointPolygonTest(poly, tuple(map(float, coords[:2])), True) < 0:
                self.rejection_reason = "Grasp-centre projection is outside the active paper boundary"
                self.cache[key] = (False, self.rejection_reason)
                return False
        # Apply the same collision-object poses in one RoboDK request.
        items, poses = [], []
        mount = camera_mount_fk(q)
        for item, _ in self.moving:
            frame = mount if item == self.camera_item else flange
            items.append(item)
            poses.append(robomath.Mat((self.base_world @ frame).tolist()))
        self.rdk.setPosesAbs(items, poses)
        valid = self.rdk.Collisions() == 0
        if not valid:
            pairs = self.rdk.CollisionPairs()
            self.rejection_reason = "RoboDK collision pairs: " + "; ".join(
                "%s [link %s] / %s [link %s]" % (a.Name(), ia, b.Name(), ib)
                for a, b, ia, ib in pairs)
        self.cache[key] = (valid, self.rejection_reason)
        return valid

    def explain_invalid(self, q):
        self.cache.pop(tuple(np.round(np.asarray(q), 6)), None)
        self.valid(q)
        return self.rejection_reason or "Invalid configuration; no detailed reason returned"

    def samples(self, a, b):
        count = max(1, int(math.ceil(np.max(np.abs(np.asarray(b)-a))/self.step)))
        return [np.asarray(a)+(np.asarray(b)-a)*i/count for i in range(count+1)]

    def edge(self, a, b):
        return all(self.valid(q) for q in self.samples(a, b))

    def route(self, start, goal):
        with TIMINGS.phase("planner.route.total"):
            return self._route(start, goal)

    def _route(self, start, goal):
        start, goal = np.asarray(start), np.asarray(goal)
        if not self.valid(start):
            raise ValueError("Starting configuration rejected: " + self.explain_invalid(start))
        if not self.valid(goal):
            raise ValueError("Target configuration rejected: " + self.explain_invalid(goal))
        if self.edge(start, goal):
            self.last_rrt_candidates = 0
            return [start.tolist(), goal.tolist()]
        nodes, parents = [start], [-1]
        step = self.setup["planning"]["rrt_step_deg"]
        for k in range(self.setup["planning"]["rrt_iterations"]):
            if k % 100 == 0:
                log("Planning joint route: %d candidates" % k)
            target = goal if k % 5 == 0 else self.rng.uniform(self.low, self.high)
            index = int(np.argmin([np.linalg.norm(q-target) for q in nodes]))
            d = target-nodes[index]
            length = max(float(np.linalg.norm(d)), 1e-10)
            if length <= 1e-9:
                continue
            q = nodes[index]+d*min(1.0, step/length)
            if not self.edge(nodes[index], q):
                continue
            nodes.append(q)
            parents.append(index)
            if self.edge(q, goal):
                self.last_rrt_candidates = k+1
                path, index = [goal], len(nodes)-1
                while index >= 0:
                    path.append(nodes[index])
                    index = parents[index]
                return [p.tolist() for p in reversed(path)]
        self.last_rrt_candidates = self.setup["planning"]["rrt_iterations"]
        raise ValueError("No collision-checked joint route found. No fixed lift/return fallback is used.")

    def ik_candidates(self, pose, seed, limit=None):
        """Validated IK solutions for pose, closest to seed first.

        Every candidate passes the same model/workspace/collision check, the
        controller OutRange check and the J2P position/angle tolerance as before.
        limit=1 stops at the first accepted solution, which is the old ik()
        behaviour used by the descent, framing and response probes.
        """
        model_target = pose @ inv(self.T_model_physical)
        allq = np.asarray(self.robot.SolveIK_All(robomath.Mat(model_target.tolist())).rows)
        if allq.size == 0:
            raise NoIKSolution("Target is outside inverse-kinematic reach")
        require(allq.ndim == 2 and allq.shape[0] >= 6 and allq.shape[1] > 0,
                "Target is outside inverse-kinematic reach")
        candidates = list(allq[:6].T)
        candidates.sort(key=lambda q: np.linalg.norm(q-seed))
        rejected, accepted = [], []
        for q in candidates:
            if not self.valid(q):
                rejected.append(self.rejection_reason)
                continue
            code = self.controller.range_code_cached(q)
            if code != 0:
                rejected.append("Controller OutRange=%s at joints %s" % (code, q.tolist()))
                continue
            real_pose = self.controller.joint_pose_cached(q)
            mm = float(np.linalg.norm(real_pose[:3, 3]-pose[:3, 3]))
            deg = rotation_error(real_pose, pose)
            if (mm <= self.setup["planning"]["position_tolerance_mm"] and
                    deg <= self.setup["planning"]["angle_tolerance_deg"]):
                accepted.append(q)
                if limit is not None and len(accepted) >= limit:
                    return accepted
                continue
            rejected.append("Controller FK differs from target: %.3f mm / %.3f deg" % (mm, deg))
        if accepted:
            return accepted
        raise ValueError("All %d IK solutions rejected: %s" %
                         (len(candidates), " | ".join(dict.fromkeys(rejected))))

    def ik(self, pose, seed):
        return self.ik_candidates(pose, seed, limit=1)[0]

    def route_to_ik(self, pose, seed):
        """IK target plus path, preferring a solution whose direct path is clear.

        Only the IK solution nearest the seed used to be routed, and if its
        straight joint path was blocked the RRT searched a heavily constrained
        space (1600 candidates, ~170 s) before failing. Candidates here pass
        exactly the same checks, every direct path is validated with the same
        edge() sampling, then a checked midpoint route is tried, and only if
        that fails is the nearest solution routed with the existing RRT.
        """
        candidates = self.ik_candidates(pose, seed)
        blocked = []
        for q in candidates:
            if self.edge(seed, q):
                self.planning_notes.append({"kind": "direct_path", "candidates": len(candidates),
                                            "found": True})
                return q, [np.asarray(seed).tolist(), q.tolist()]
            blocked.append(self.rejection_reason)
        reasons = "; ".join(dict.fromkeys(blocked))[:600]
        via = self.route_via_midpoint(pose, seed, candidates[0])
        if via is not None:
            self.planning_notes.append({"kind": "midpoint_route", "candidates": len(candidates),
                                        "blocked": reasons})
            log("Direct joint path is blocked for all %d validated IK solution(s): %s. "
                "A checked midpoint route connects the same endpoints; using it instead of the "
                "RRT search." % (len(candidates), reasons))
            return candidates[0], via
        self.planning_notes.append({"kind": "rrt_fallback", "candidates": len(candidates),
                                    "blocked": reasons})
        log("Direct joint path is blocked for all %d validated IK solution(s): %s. "
            "Routing to the nearest solution with the existing RRT search." %
            (len(candidates), reasons))
        try:
            path = self.route(seed, candidates[0])
        except ValueError as exc:
            self.planning_notes[-1].update(found=False, rrt_candidates=self.last_rrt_candidates,
                                           error=str(exc)[:300])
            raise ValueError("%s (direct paths blocked: %s)" % (exc, reasons)) from exc
        self.planning_notes[-1].update(found=True, rrt_candidates=self.last_rrt_candidates)
        return candidates[0], path

    def route_via_midpoint(self, pose, seed, qa):
        """Two-leg fallback through a checked Cartesian midpoint.

        A straight joint path can cross a collision even when the Cartesian
        corridor between the same two poses is clear. The midpoint pose is
        inverse-kinematically solved and both legs are validated with the same
        edge() sampling; if anything fails, the existing RRT search still runs.
        """
        start = self.fk(seed)
        delta = rotation_vector(start[:3, :3].T @ np.asarray(pose)[:3, :3])
        for fraction in (0.5, 0.33, 0.66):
            mid = np.eye(4)
            mid[:3, :3] = start[:3, :3] @ cv2.Rodrigues(fraction*delta)[0]
            mid[:3, 3] = start[:3, 3] + fraction*(np.asarray(pose)[:3, 3]-start[:3, 3])
            try:
                qm = self.ik(mid, seed)
            except ValueError:
                continue
            if self.edge(seed, qm) and self.edge(qm, qa):
                return [np.asarray(seed).tolist(), np.asarray(qm).tolist(), np.asarray(qa).tolist()]
        return None

    def preflight(self, path, label=None):
        with TIMINGS.phase("planner.preflight.total"):
            self._preflight(path, label)
        # Receipt for execute(): reusable only while the scene signature matches.
        self.preflighted[self._path_key(path)] = self.scene_signature()

    def _preflight(self, path, label=None):
        started = reported = time.monotonic()
        checked = 0
        if label:
            log("Preflight %s: checking collision/controller samples before motion." % label)
        for a, b in zip(path, path[1:]):
            for q in self.samples(a, b):
                check_stop()
                with TIMINGS.phase("planner.preflight.valid"):
                    require(self.valid(q), "Route rejected: " + self.rejection_reason)
                with TIMINGS.phase("planner.preflight.controller_range"):
                    code = self.controller.range_code_cached(q)
                require(code == 0, "Controller rejects sampled route joints %s: OutRange=%s" % (q.tolist(), code))
                checked += 1
                now = time.monotonic()
                if label and now-reported >= 5:
                    log("Preflight %s: %d samples checked; elapsed %.1f s." % (label, checked, now-started))
                    reported = now
            actual_fk = self.controller.joint_pose_cached(b)
            model_fk = self.fk(b)
            require(np.linalg.norm(actual_fk[:3, 3]-model_fk[:3, 3]) <=
                    self.setup["planning"]["position_tolerance_mm"] and
                    rotation_error(actual_fk, model_fk) <= self.setup["planning"]["angle_tolerance_deg"],
                    "Controller/RoboDK kinematics disagree on route")
        if label:
            log("Preflight %s passed: %d samples in %.1f s." % (label, checked, time.monotonic()-started))

    def execute(self, path, label, report_progress=False):
        # A live re-check still runs unless this exact path was already checked
        # against the identical collision scene. plan_pick() supplies that
        # receipt; the lift's first check with the held payload has no receipt
        # and therefore always runs here. See scene_signature() for the inputs.
        if self.preflighted.get(self._path_key(path)) == self.scene_signature():
            log("%s: identical path and collision scene to the planning check; "
                "reusing that preflight instead of repeating it live." % label)
        else:
            self.preflight(path, label if report_progress else None)
        for i, (a, b) in enumerate(zip(path, path[1:]), 1):
            log("%s: joint waypoint %d/%d" % (label, i, len(path)-1))
            self.controller.move_joint(np.asarray(a), np.asarray(b))
        state = self.controller.stable(powered=True)
        self.sync_check(state)
        # Verified post-move state. Callers that ignore it are unaffected; a
        # collection image uses it instead of repeating controller.stable().
        return state

    def close(self):
        # Generated scene geometry is temporary. Existing objects are never deleted.
        for item in reversed(self.items):
            try:
                item.Delete()
            except Exception:
                pass
        # Planning disabled rendering; restore the station view for the operator.
        try:
            self.rdk.Render(True)
        except Exception as exc:
            log("Could not restore RoboDK rendering: " + str(exc))


def project_points(h_camera_board, obj, k, d):
    cp = points(h_camera_board, obj)
    require(np.all(cp[:, 2] > 0), "Board lies behind camera (pose ambiguity)")
    return cv2.projectPoints(np.asarray(obj, float), rotation_vector(h_camera_board[:3, :3]),
                             h_camera_board[:3, 3], k, d)[0].reshape(-1, 2)


def pnp(obj, pix, k, d):
    ok, r, t = cv2.solvePnP(np.asarray(obj, float), np.asarray(pix, float), k, d,
                           flags=cv2.SOLVEPNP_ITERATIVE)
    require(ok, "Board pose solve failed")
    h = transform(cv2.Rodrigues(r)[0], t)
    require(np.all(points(h, obj)[:, 2] > 0), "Invalid planar pose branch")
    return h


def validate_calibration_intrinsics(k, d):
    require(np.isfinite(k).all() and np.isfinite(d).all(), "Nonfinite camera solution")
    require(200 < k[0, 0] < 10000 and 200 < k[1, 1] < 10000 and
            0 < k[0, 2] < 1920 and 0 < k[1, 2] < 1080, "Implausible intrinsics")
    # Reject radial folds throughout the full sensor, not only near the board.
    corners = np.array([[0, 0], [1919, 0], [1919, 1079], [0, 1079]], float)
    normalized = cv2.undistortPoints(corners.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    require(np.isfinite(normalized).all(), "Nonfinite normalized sensor corners")
    r2 = np.linspace(0, np.max(np.sum(normalized*normalized, axis=1))*1.2, 300)
    k1, k2, _, _, k3 = d.reshape(-1)[:5]
    require(np.all(1+3*k1*r2+5*k2*r2**2+7*k3*r2**3 > 0), "Distortion folds inside sensor")


def refine_fixed_board(bf, objects, images, fc, bb, k, d):
    """Refine one lens, J5-camera transform and stationary board using fit views only."""
    initial = np.r_[rotation_vector(fc[:3, :3]), fc[:3, 3],
                    rotation_vector(bb[:3, :3]), bb[:3, 3],
                    k[0, 0], k[1, 1], k[0, 2], k[1, 2], d.reshape(-1)]
    # Optimize comparable increments across radians, millimetres, pixels and distortion.
    scales = np.array([.01, .01, .01, 10, 10, 10]*2 +
                      [100, 100, 100, 100, 1, 10, .01, .01, 100])
    fit_views = [(mount, obj.astype(float), pix.reshape(-1, 2).astype(float))
                 for mount, obj, pix in zip(bf, objects, images)]

    def unpack(offset):
        x = initial+offset*scales
        fc = transform(cv2.Rodrigues(x[:3])[0], x[3:6])
        bb = transform(cv2.Rodrigues(x[6:9])[0], x[9:12])
        fx, fy, cx, cy = x[12:16]
        return fc, bb, np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), x[16:]

    def residual(offset):
        fc, bb, k, d = unpack(offset)
        if not (200 < k[0, 0] < 10000 and 200 < k[1, 1] < 10000 and
                0 < k[0, 2] < 1920 and 0 < k[1, 2] < 1080):
            return None
        errors = []
        for mount, obj, pix in fit_views:
            predicted = inv(mount @ fc) @ bb
            if np.any(points(predicted, obj)[:, 2] <= 0):
                return None
            errors.append((project_points(predicted, obj, k, d)-pix).ravel())
        errors = np.concatenate(errors)
        return errors if np.isfinite(errors).all() else None

    # Small damped least-squares problem with a numerical Jacobian. A rejected
    # trial only increases damping. Held-out observations never enter this fit.
    offset = np.zeros_like(initial)
    errors = residual(offset)
    require(errors is not None, "Invalid fixed-board refinement initializer")
    start_cost = cost = float(errors @ errors)
    damping = 1e-3
    epsilon = 1e-5
    perturbations = epsilon*np.eye(offset.size)
    jacobian = None
    for _ in range(80):
        check_stop()
        if jacobian is None:
            columns = []
            for step in perturbations:
                plus, minus = residual(offset+step), residual(offset-step)
                require(plus is not None and minus is not None,
                        "Fixed-board refinement cannot evaluate a physical Jacobian")
                columns.append((plus-minus)/(2*epsilon))
            jacobian = np.column_stack(columns)
            normal = jacobian.T @ jacobian
            gradient = jacobian.T @ errors
            diagonal = np.diag(np.maximum(np.diag(normal), 1e-12))
        step = np.linalg.solve(normal+damping*diagonal, -gradient)
        if np.linalg.norm(step) < 1e-7:
            break
        trial = residual(offset+step)
        trial_cost = float(trial @ trial) if trial is not None else math.inf
        if trial_cost < cost:
            offset += step
            errors, cost = trial, trial_cost
            damping = max(damping*.25, 1e-12)
            jacobian = None
        else:
            damping *= 4
    fc, bb, k, d = unpack(offset)
    validate_calibration_intrinsics(k, d)
    fixed_rms = math.sqrt(2*cost/errors.size)
    # The existing train gate measures optical reprojection with each image
    # free to solve its own pose. Recompute it using the final refined optics.
    view_squares = []
    for _, obj, pix in fit_views:
        h = pnp(obj, pix, k, d)
        err = project_points(h, obj, k, d)-pix
        view_squares.append(np.sum(err*err, axis=1))
    optical_rms = float(np.sqrt(np.mean(np.concatenate(view_squares))))
    per_view = np.array([np.sqrt(np.mean(sq)) for sq in view_squares])
    quality = {"initial_fixed_board_rms_px": math.sqrt(2*start_cost/errors.size),
               "refined_fixed_board_rms_px": fixed_rms}
    return fc, bb, k, d, optical_rms, per_view, quality


def fit_calibration(records, setup, folder):
    board, _ = get_board()
    good = [r for r in records if r.get("accepted")]
    train = [r for r in good if r["role"] == "fit"]
    checks = [r for r in good if r["role"] == "check"]
    q = setup["quality"]
    require(len(train) >= q["min_fit"] and len(checks) >= q["min_check"],
            "Insufficient independent views: %d fit / %d check" % (len(train), len(checks)))
    require(len({r["image_sha256"] for r in good}) == len(good),
            "Repeated JPEGs across different poses; possible cached images")
    objects = [np.asarray(board.chessboardCorners)[r["corner_ids"]].astype(np.float32) for r in train]
    images = [np.asarray(r["corners_px"], np.float32).reshape(-1, 1, 2) for r in train]
    initial_rms, k, d, rv, tv, std_i, _, _ = cv2.calibrateCameraExtended(
        objects, images, (1920, 1080), None, None, flags=0,
        criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_COUNT, 150, 1e-10))
    validate_calibration_intrinsics(k, d)
    bf = [rigid(r["T_base_camera_mount"]) for r in train]
    cb = [transform(cv2.Rodrigues(r)[0], t) for r, t in zip(rv, tv)]
    relative_rotations = np.array([rotation_vector(a[:3, :3].T @ b[:3, :3])
                                  for a, b in itertools.combinations(bf, 2)])
    singular = np.linalg.svd(relative_rotations, compute_uv=False)
    require(singular[1] >= 0.4 and singular[1]/singular[0] >= 0.15,
            "Hand-eye rotation excitation is insufficient: rotate about at least two nonparallel axes")
    span = np.ptp(np.array([h[:3, 3] for h in bf]), axis=0)
    require(np.linalg.norm(span) >= 25, "Camera positions have insufficient translation variation")
    normal_rot = np.array([rotation_vector(cb[0][:3, :3].T @ h[:3, :3]) for h in cb])
    require(np.max(np.linalg.norm(normal_rot[:, :2], axis=1))*180/math.pi >= 15,
            "Board tilt variation below 15 degrees; intrinsics would be weakly constrained")
    r_fc, t_fc = cv2.calibrateHandEye([h[:3, :3] for h in bf], [h[:3, 3] for h in bf],
                                     [h[:3, :3] for h in cb], [h[:3, 3] for h in cb],
                                     method=cv2.CALIB_HAND_EYE_PARK)
    fc = rigid(transform(r_fc, t_fc))
    require(np.linalg.norm(fc[:3, 3]) < 350, "Hand-eye translation implausible for wrist mount")
    bb = mean_transform([a @ fc @ b for a, b in zip(bf, cb)])
    fc, bb, k, d, rms, per_view, refinement = refine_fixed_board(bf, objects, images, fc, bb, k, d)
    require(np.linalg.norm(fc[:3, 3]) < 350, "Refined hand-eye translation implausible for wrist mount")
    n = bb[:3, 2].copy()
    if n[2] < 0:
        n = -n
    require(n[2] > math.cos(math.radians(10)), "Board/table is not approximately horizontal in base frame")
    table_p = bb[:3, 3]-float(setup["board_thickness_mm"])*n
    table_z = float(setup["table_initial_z_mm"])
    table_uncertainty = float(setup["table_initial_uncertainty_mm"])
    table_height_passed = abs(table_p[2]-table_z) <= table_uncertainty
    check_results = []
    for rec in checks:
        obj = np.asarray(board.chessboardCorners)[rec["corner_ids"]].astype(float)
        pix = np.asarray(rec["corners_px"], float)
        bfi = rigid(rec["T_base_camera_mount"])
        predicted = inv(bfi @ fc) @ bb
        measured = pnp(obj, pix, k, d)
        pred_px = project_points(predicted, obj, k, d)
        reproj = np.linalg.norm(pred_px-pix, axis=1)
        board_measured = bfi @ fc @ measured
        # Held-out ray intersection with the FIT board plane: measurable metric error.
        cam = bfi @ fc
        rays = cv2.undistortPoints(pix.reshape(-1, 1, 2), k, d).reshape(-1, 2)
        rays = np.column_stack([rays, np.ones(len(rays))]) @ cam[:3, :3].T
        denom = rays @ n
        require(np.all(np.abs(denom) > 1e-6), "Validation ray is parallel to table")
        depths = ((bb[:3, 3]-cam[:3, 3]) @ n)/denom
        hit = cam[:3, 3]+depths[:, None]*rays
        metric = np.linalg.norm(hit-points(bb, obj), axis=1)
        check_results.append({"label": rec["label"], "rms_px": float(np.sqrt(np.mean(reproj**2))),
                              "max_px": float(reproj.max()), "max_corner_mm": float(metric.max()),
                              "board_origin_mm": float(np.linalg.norm(board_measured[:3, 3]-bb[:3, 3])),
                              "board_angle_deg": rotation_error(board_measured, bb)})
    allpix = np.concatenate([np.asarray(r["corners_px"], np.float32) for r in train])
    hull = cv2.convexHull(allpix).reshape(-1, 2)
    coverage = float(cv2.contourArea(hull)/(1920*1080))
    spans = np.ptp(allpix, axis=0)/[1920, 1080]
    quality = {"fit_count": len(train), "check_count": len(checks), "train_rms_px": float(rms),
               "per_fit_rms_px": per_view.reshape(-1).tolist(),
               "initial_train_rms_px": float(initial_rms),
               "initial_intrinsic_std_deviations": std_i.reshape(-1).tolist(),
               "fixed_board_refinement": refinement,
               "table_height": {"measured_z_mm": float(table_p[2]),
                                "allowed_z_mm": [table_z-table_uncertainty, table_z+table_uncertainty],
                                "passed": bool(table_height_passed)},
               "rotation_excitation_singular_values_rad": singular.tolist(),
               "translation_span_mm": span.tolist(), "image_coverage_fraction": coverage,
               "image_coverage_span_xy": spans.tolist(), "held_out": check_results,
               "max_check_rms_px": max(c["rms_px"] for c in check_results),
               "max_check_mm": max(c["max_corner_mm"] for c in check_results),
               "max_check_angle_deg": max(c["board_angle_deg"] for c in check_results),
               "absolute_grasp_accuracy_measured": False}
    quality["passed"] = bool(table_height_passed and rms <= q["train_rms_px"] and
                             quality["max_check_rms_px"] <= q["check_rms_px"] and
                             quality["max_check_mm"] <= q["check_max_mm"] and
                             quality["max_check_angle_deg"] <= q["check_max_angle_deg"] and
                             coverage >= q["coverage_fraction"] and
                             np.all(spans >= q["coverage_span_xy"]))
    # Contact-centre TCP chosen so a vertical grasp leaves the CAD finger tips clear.
    tcp_z = setup["finger_tip_z_mm"]-(setup["cube_height_mm"]/2-setup["tip_clearance_mm"])
    fg = transform(t=[0, 0, tcp_z])
    result = {"schema": SCHEMA, "created_utc": stamp(), "units": {"length": "mm", "angle": "degree"},
              "transform_convention": "T_A_B maps B points to A; F=physical J6; M=DENSO ROS J5; B=robot base",
              "board": BOARD, "image_size_wh": [1920, 1080],
              "intrinsics": {"K": k.tolist(), "distortion_opencv_k1_k2_p1_p2_k3": d.reshape(-1).tolist(),
                             "model": "pinhole_brown_5", "opencv_version": cv2.__version__},
              "hand_eye": {"mounting_link": 5, "mount_frame": "DENSO_ROS_J5",
                           "T_mount_camera": fc.tolist(),
                           "method": "OpenCV PARK initializer plus fixed-board reprojection refinement"},
              "table": {"T_base_board": bb.tolist(), "point_base_mm": table_p.tolist(),
                        "normal_base": n.tolist(), "printed_surface_above_table_mm": setup["board_thickness_mm"]},
              "grasp_tcp": {"T_flange_grasp": fg.tolist(), "source": setup["gripper_geometry_source"],
                            "method": "stock CAD plus chosen tip clearance; NOT camera calibration",
                            "finger_tip_z_mm": setup["finger_tip_z_mm"],
                            "tip_clearance_mm": setup["tip_clearance_mm"],
                            "physical_grasp_validation_performed": False},
              "quality": quality, "calibrated_image_hull_px": hull.tolist(),
              "session_folder": str(Path(folder).resolve())}
    write_json(Path(folder)/"calibration_candidate.json", result)
    return result


def table_basis(cal):
    n = np.asarray(cal["table"]["normal_base"])
    x = np.asarray(cal["table"]["T_base_board"])[:3, 0]
    x = x-n*np.dot(x, n)
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    return np.column_stack([x, y, n]), np.asarray(cal["table"]["point_base_mm"])


def camera_parameters(cal, state):
    k = np.asarray(cal["intrinsics"]["K"], float)
    d = np.asarray(cal["intrinsics"]["distortion_opencv_k1_k2_p1_p2_k3"], float)
    require(cal["hand_eye"].get("mount_frame") == "DENSO_ROS_J5",
            "Camera mounting frame changed; run calibrate with this version")
    bc = camera_mount_fk(state["joints"]) @ rigid(cal["hand_eye"]["T_mount_camera"])
    return k, d, bc


def ray_hit(pixel, bc, k, d, point, normal):
    uv = cv2.undistortPoints(np.asarray(pixel, float).reshape(1, 1, 2), k, d).reshape(2)
    ray = bc[:3, :3] @ np.r_[uv, 1]
    denom = np.dot(ray, normal)
    require(abs(denom) > 1e-6, "Viewing ray parallel to table")
    distance = np.dot(point-bc[:3, 3], normal)/denom
    require(distance > 0, "Table intersection behind camera")
    return bc[:3, 3]+distance*ray


def register_printed_paper(camera, controller, cal, folder):
    """Register the fixed untrimmed Letter print and save the projected outline."""
    frame, record, prefix = camera.frame("paper_boundary", controller)
    k, d, bc = camera_parameters(cal, record["state_after"])
    axes, origin = table_basis(cal)
    w, height = PRINTED_PAPER_LAYOUT["size_mm"]
    x, y = PRINTED_PAPER_LAYOUT["pattern_lower_left_from_page_lower_left_mm"]
    board_corners = np.array([[-x, -y, 0], [w-x, -y, 0],
                              [w-x, height-y, 0], [-x, height-y, 0]], float)
    hits = points(rigid(cal["table"]["T_base_board"]), board_corners)
    hits -= float(cal["table"]["printed_surface_above_table_mm"])*axes[:, 2]
    outline_pixels = project_points(inv(bc), hits, k, d)
    xy = ((hits-origin) @ axes)[:, :2].astype(np.float32)
    require(cv2.isContourConvex(xy.reshape(-1, 1, 2)),
            "Fixed paper outline is not convex")
    lengths = np.linalg.norm(np.roll(xy, -1, axis=0)-xy, axis=1)
    sizes = sorted([float((lengths[0]+lengths[2])/2), float((lengths[1]+lengths[3])/2)])
    require(abs(sizes[0]-215.9) <= 15 and abs(sizes[1]-279.4) <= 15,
            "Fixed outline does not agree with Letter paper dimensions (215.9 x 279.4 mm): " + str(sizes))
    require(abs(lengths[0]-lengths[2]) < 10 and abs(lengths[1]-lengths[3]) < 10,
            "Opposite paper edges disagree; inspect print layout and calibration")
    fg = rigid(cal["grasp_tcp"]["T_flange_grasp"])
    # The current pose can be the acquired framing pose, distinct from saved VIEW.
    view_g = (controller.physical_pose(controller.cfg["observation"]["pose"]) @ fg)[:3, 3]
    view_xy = axes.T @ (view_g-origin)
    require(cv2.pointPolygonTest(xy, tuple(map(float, view_xy[:2])), True) >= 0,
            "Saved VIEW projects outside the registered paper; cannot return while enforcing the paper boundary")
    drawn = frame.copy()
    require(np.isfinite(outline_pixels).all() and np.max(np.abs(outline_pixels)) < 100000,
            "Fixed paper projection is invalid")
    cv2.polylines(drawn, [np.rint(outline_pixels).astype(np.int32)], True, (0, 255, 255), 3)
    show_image(drawn, "PAPER BOUNDARY | fixed Letter print")
    cv2.imwrite(str(prefix.with_name(prefix.name+"_outline.png")), drawn)
    workspace = {"method": "calibrated board plus fixed printed-page registration",
                 "polygon_table_xy_mm": xy.tolist(), "axes_base": axes.tolist(),
                 "origin_base_mm": origin.tolist(), "corners_base_mm": hits.tolist(),
                 "paper_size_mm": [215.9, 279.4], "size_source": "fixed Letter print registration",
                 "size_mm": sizes, "selected_pixels": [], "outline_pixels": outline_pixels.tolist(),
                 "cube_edge_clearance_mm": 5.0,
                 "scope": "grasp centre along pick routes and full cube footprint at grasp",
                 "print_registration": {"layout": PRINTED_PAPER_LAYOUT,
                                        "pdf_sha256": PRINTABLE_BOARD_SHA256,
                                        "fixed_setup": True,
                                        "unseen_edges_independently_measured": False}}
    write_json(Path(folder)/"paper_workspace.json", workspace)
    return workspace


def contour_samples(poly, count=100):
    a = np.asarray(poly, float).reshape(-1, 2)
    b = np.roll(a, -1, axis=0)
    lengths = np.linalg.norm(b-a, axis=1)
    require(np.all(lengths > 1e-8), "Degenerate object outline")
    result = []
    for p, v, length in zip(a, b-a, lengths):
        n = max(2, round(count*length/lengths.sum()))
        result.extend(p + v*f for f in np.linspace(0, 1, n, endpoint=False))
    return np.asarray(result)


def distance_to_polygon(p, poly):
    a = np.asarray(poly, float)
    v = np.roll(a, -1, axis=0)-a
    t = np.clip(np.sum((p[:, None, :]-a)*v, axis=2)/np.sum(v*v, axis=1), 0, 1)
    delta = p[:, None, :]-a-t[:, :, None]*v
    return np.sqrt(np.min(np.sum(delta*delta, axis=2), axis=1))


def locate_cube(frame, vision, cal, state, setup, diagnostic_prefix=None):
    glare_profile = PICK_OBJECT_PROFILE["glare_repair"]
    detection = detect(frame, vision, glare_profile)
    require(detection["status"] == "single_candidate", "Cube detection: " + detection["status"])
    cand = detection["candidates"][0]
    # Candidate proposals and outline fitting must use the same recovered mask.
    mask = blue_mask(frame, vision, glare_profile)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centre = tuple(map(float, cand["centre_px"]))
    contours = [c for c in contours if cv2.pointPolygonTest(cv2.convexHull(c), centre, False) >= 0]
    require(len(contours) == 1, "Could not isolate the blue cube outline")
    contour = contours[0]
    hull = cv2.convexHull(contour).reshape(-1, 2).astype(float)
    calibrated_hull = np.asarray(cal["calibrated_image_hull_px"], np.float32)
    # Report how far outside and where, so the operator can nudge the cube or
    # decide to recalibrate with coverage there. The gate itself is unchanged.
    margin_px = [float(cv2.pointPolygonTest(calibrated_hull, tuple(map(float, p)), True))
                 for p in hull]
    worst = int(np.argmin(margin_px))
    covered = calibrated_hull.reshape(-1, 2)
    require(min(margin_px) >= 0,
            "Cube lies outside the image region covered by calibration observations: outline "
            "point (%.0f, %.0f) px is %.0f px outside the calibrated hull (covered region "
            "x %.0f..%.0f, y %.0f..%.0f px). Move the cube into the covered region, or "
            "recalibrate with collection coverage there; this gate was not relaxed." %
            (hull[worst][0], hull[worst][1], -min(margin_px),
             covered[:, 0].min(), covered[:, 0].max(), covered[:, 1].min(), covered[:, 1].max()))
    solidity = cv2.contourArea(contour)/cv2.contourArea(hull.astype(np.float32))
    require(solidity >= PICK_OBJECT_PROFILE["min_outline_solidity"],
            "Blue outline has more missing support than the perforated cube profile allows")
    require(np.all(hull.min(0) > [8, 8]) and np.all(hull.max(0) < [1911, 1071]),
            "Cube is clipped or too near the image edge")
    observed = contour_samples(hull)
    k, d, bc = camera_parameters(cal, state)
    axes, origin = table_basis(cal)
    width, height = float(setup["cube_width_mm"]), float(setup["cube_height_mm"])
    guess = axes.T @ (ray_hit(centre, bc, k, d, origin+axes[:, 2]*height, axes[:, 2])-origin)
    cb = inv(bc)
    # The blue contour observes the top face. Dark side walls are absent from
    # this mask, especially off-centre. Collision geometry still uses the full
    # 25 mm object in set_target_cube and the carried-payload envelope.
    local_face = np.array(list(itertools.product([-width/2, width/2], [-width/2, width/2], [height])))

    def model(v):
        c, s = math.cos(v[2]), math.sin(v[2])
        r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        vertices = (local_face @ r.T + [v[0], v[1], 0]) @ axes.T+origin
        cam_vertices = points(cb, vertices)
        if np.any(cam_vertices[:, 2] <= 0):
            return None
        px = cv2.projectPoints(cam_vertices, np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)
        return cv2.convexHull(px.astype(np.float32)).reshape(-1, 2).astype(float)

    def cost(v):
        poly = model(v)
        if poly is None:
            return 1e12
        a = distance_to_polygon(observed, poly)
        b = distance_to_polygon(contour_samples(poly), hull)
        return float((np.mean(a*a)+np.mean(b*b))/2)

    solutions = []
    for yaw in np.linspace(0, math.pi/2, 8, endpoint=False):
        v = np.array([guess[0], guess[1], yaw])
        best = cost(v)
        for scale in [4, 2, 1, .5, .2, .08, .03]:
            step = np.array([scale, scale, math.radians(scale*2)])
            for _ in range(30):
                choices = [(cost(v+sign*step[i]*np.eye(3)[i]), v+sign*step[i]*np.eye(3)[i])
                           for i in range(3) for sign in (-1, 1)]
                score, candidate = min(choices, key=lambda item: item[0])
                if score >= best-1e-8:
                    break
                best, v = score, candidate
        solutions.append((best, v))
        check_stop()
    score, fit = min(solutions, key=lambda item: item[0])
    projected = model(fit)
    _, intersection = cv2.intersectConvexConvex(projected.astype(np.float32), hull.astype(np.float32))
    inter_area = cv2.contourArea(intersection) if intersection is not None else 0.0
    union = cv2.contourArea(projected.astype(np.float32))+cv2.contourArea(hull.astype(np.float32))-inter_area
    iou = inter_area/union
    # Rounded faces and an approximate ruler dimension cannot reproduce a
    # sharp box to two pixels. Bound discrepancy in table millimetres using
    # the least sensitive axis of the calibrated local projection instead.
    probe_base = origin+np.array([[fit[0], fit[1], height],
                                  [fit[0]+1, fit[1], height],
                                  [fit[0], fit[1]+1, height]])@axes.T
    probe_px = project_points(cb, probe_base, k, d)
    pixels_per_mm = float(np.linalg.svd((probe_px[1:]-probe_px[0]).T, compute_uv=False)[-1])
    require(pixels_per_mm > 0, "Degenerate calibrated image scale at cube")
    fit_rms_mm = math.sqrt(score)/pixels_per_mm
    fit_passed = (fit_rms_mm <= PICK_OBJECT_PROFILE["max_fit_rms_mm"] and
                  iou >= PICK_OBJECT_PROFILE["min_silhouette_iou"])
    # Save both outlines even when size verification stops the run. Never
    # substitute a fitted object size for the user's confirmed dimensions.
    drawn = overlay(frame, detection)
    # The helper's generic "detection setup only" banner is misleading in Pick.
    cv2.rectangle(drawn, (0, 0), (drawn.shape[1], 80), (0, 0, 0), -1)
    cv2.putText(drawn, "CUBE IMAGE CHECK | green observed; magenta model",
                (15, 35), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
    cv2.putText(drawn, "RMS %.2f mm | IoU %.3f | planning/motion progress is in the console" %
                (fit_rms_mm, iou), (15, 65), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
    cv2.polylines(drawn, [np.rint(hull).astype(np.int32)], True, (0, 255, 0), 2)
    cv2.polylines(drawn, [np.rint(projected).astype(np.int32)], True, (255, 0, 255), 2)
    shape_check = {"profile": PICK_OBJECT_PROFILE, "outline_solidity": float(solidity),
                   "assumed_outer_size_mm": [width, width, height],
                   "observed_span_px": np.ptp(hull, axis=0).tolist(),
                   "predicted_span_px": np.ptp(projected, axis=0).tolist(),
                   "fit_rms_px": math.sqrt(score), "silhouette_iou": iou,
                   "fit_rms_mm": fit_rms_mm,
                   "local_min_pixels_per_mm": pixels_per_mm, "passed": bool(fit_passed)}
    if diagnostic_prefix is not None:
        prefix = Path(diagnostic_prefix)
        write_json(prefix.with_name(prefix.name+"_shape_check.json"), shape_check)
        require(cv2.imwrite(str(prefix.with_name(prefix.name+"_shape_check.png")), drawn),
                "Could not save cube shape diagnostic image")
    require(fit_passed,
            "Known-size 3D cube does not fit blue outline: %.2f px / %.2f mm RMS, IoU %.3f. "
            "See shape_check image/JSON for observed and predicted size; "
            "verify outer dimensions, support height and camera scale." % (math.sqrt(score), fit_rms_mm, iou))
    for alternative_score, alt in solutions:
        if alternative_score <= score+.10:
            require(np.linalg.norm(alt[:2]-fit[:2]) < .8 and
                    abs((alt[2]-fit[2]+math.pi/4) % (math.pi/2)-math.pi/4) < math.radians(4),
                    "Cube pose is visually ambiguous")
    point = origin+axes @ [fit[0], fit[1], height/2]
    edge = axes @ [math.cos(fit[2]), math.sin(fit[2]), 0]
    require("workspace" in cal, "Calibrate and mark the paper boundary before picking")
    poly = np.asarray(cal["workspace"]["polygon_table_xy_mm"], np.float32)
    c, s = math.cos(fit[2]), math.sin(fit[2])
    r2 = np.array([[c, -s], [s, c]])
    footprint = np.array(list(itertools.product([-width/2, width/2], repeat=2))) @ r2.T + fit[:2]
    require(all(cv2.pointPolygonTest(poly, tuple(map(float, p)), True) >=
                cal["workspace"]["cube_edge_clearance_mm"] for p in footprint),
            "Cube footprint is outside the paper or within 5 mm of its edge")
    show_image(drawn, "FRESH SNAPSHOT | cube top-face fit %.2f px | no motion yet" % math.sqrt(score))
    return {"centre_base_mm": point.tolist(), "closing_axis_base": edge.tolist(),
            "yaw_table_rad": float(fit[2]), "fit_rms_px": math.sqrt(score), "silhouette_iou": iou,
            "fit_rms_mm": fit_rms_mm,
            "projected_outline_px": projected.tolist(), "shape_check": shape_check, "detection": detection}, drawn


def view_check(state, cfg):
    require(np.max(np.abs(np.asarray(state["joints"])-np.asarray(cfg["observation"]["joints"][:6]))) <= .25,
            "Not at saved VIEW joints")
    saved = pose_matrix(cfg["observation"]["pose"])
    actual = pose_matrix(state["pose"])
    require(np.linalg.norm(actual[:3, 3]-saved[:3, 3]) <= .8 and rotation_error(actual, saved) <= .4,
            "VIEW flange pose changed; controller calibration/tool/work mismatch")


def bind_calibration(cal, cfg, vision, setup, frames):
    require(cal["schema"] == SCHEMA and cal["quality"]["passed"], "No accepted J5 camera calibration; run calibrate")
    require(cal.get("board") == BOARD,
            "Calibration used a different board definition. Recalibrate with the CalibrX "
            "25 mm square / 18 mm marker sheet before picking; old scale cannot be reused.")
    require(cal["binding"]["observation"] == cfg["observation"], "Saved VIEW changed; recalibrate")
    require(cal["binding"]["frames"] == frames, "Controller frame definitions changed")
    require(cal["binding"]["vision_fingerprint"] == fingerprint(vision), "Vision settings changed; recalibrate")
    current_geometry = geometry_binding(setup)
    expected_hash = cal["binding"]["geometry_fingerprint"]
    if expected_hash != fingerprint(current_geometry):
        # Legacy files bundled object size into the camera dependency hash.
        # Authenticate their original geometry before allowing only size edits.
        recorded_geometry = cal["binding"].get("geometry")
        if recorded_geometry is None:
            source = Path(cal["session_folder"])/"start_state.json"
            require(source.is_file(), "Original calibration setup is missing; cannot verify changed object size")
            recorded_geometry = geometry_binding(read_json(source)["setup"])
        require(fingerprint(recorded_geometry) == expected_hash,
                "Recorded calibration geometry does not match its saved fingerprint")
        camera_geometry = dict(current_geometry)
        for key in ("cube_width_mm", "cube_height_mm"):
            camera_geometry[key] = recorded_geometry[key]
        require(fingerprint(camera_geometry) == expected_hash,
                "Mount/finger/board geometry changed; recalibrate")
    require(cal["binding"]["robot_ip"] == cfg["robot_ip"], "Robot changed")


def geometry_binding(setup):
    binding = {k: setup[k] for k in ("base_on_same_table", "factory_camera_mount_confirmed",
                                 "camera_mount_frame", "camera_envelope_sha256",
                                 "stock_fingers_confirmed", "finger_tip_z_mm", "tip_clearance_mm",
                                 "cube_height_mm", "cube_width_mm", "board_thickness_mm")}
    binding["gripper_body_envelope_sha256"] = GRIPPER_BODY_SHA256
    return binding


def route_to_view(planner, controller, cfg):
    start = controller.stable(powered=True)
    planner.sync_check(start)
    path = planner.route(start["joints"], cfg["observation"]["joints"][:6])
    planner.execute(path, "Return to VIEW")
    view_check(controller.stable(), cfg)
    return path


class FramingGuide:
    """Provisional image response measured in this run, not calibration.

    The model only proposes checked poses. Fresh-image verification is mandatory;
    its coefficients never enter the accepted intrinsics/hand-eye/table solution.
    """
    def __init__(self, camera, setup, folder):
        self.camera, self.setup = camera, setup
        self.folder = folder
        self.reference = self.response = None
        self.validated = False
        self.obj = np.asarray(camera.board.chessboardCorners)[:, :2].astype(np.float32)
        self.markers = np.asarray(camera.board.objPoints)[:, :, :2].astype(np.float32)
        distances = np.linalg.norm(self.obj[:, None, :]-self.markers.mean(1)[None, :, :], axis=2)
        self.neighbours = np.argsort(distances, axis=1)[:, :2]

    def fit_response(self, samples):
        # Actual measured poses, never commanded poses or retained coefficients.
        # This describes local flange commands on the checked IK branch. Images
        # and mount checks still use J5 FK; this does not attach the camera to J6.
        self.reference = rigid(samples[0]["T_base_flange"])
        x = np.array([np.r_[1, self.features(r["T_base_flange"])] for r in samples])
        y = np.array([self.measure(r)[1] for r in samples])
        scales = np.array([1, 5, 5, 5, 2, 2], float)
        coef, _, rank, singular = np.linalg.lstsq(x/scales, y, rcond=None)
        residuals = np.linalg.norm((x/scales)@coef-y, axis=1)
        response = (coef/scales[:, None])[1:].T
        write_json(self.folder/"framing_response_model.json", {
            "purpose": "provisional current-run proposals only; NOT accepted calibration",
            "source_session": Path(self.folder).name,
            "images": [r["image_file"] for r in samples],
            "reference_T_base_flange": self.reference.tolist(),
            "reference_T_base_camera_mount": samples[0]["T_base_camera_mount"],
            "features": ["base_X_mm", "base_Y_mm", "base_Z_mm", "world_Rx_deg", "world_Ry_deg"],
            "board_ray_XY_response": response.tolist(),
            "fit_rank": int(rank), "scaled_singular_values": singular.tolist(),
            "source_max_residual_board_mm": float(residuals.max()),
            "source_rms_residual_board_mm": float(np.sqrt(np.mean(residuals**2))),
            "independent_validation_required": True})
        require(rank == 6 and singular[-1] > 0 and singular[0]/singular[-1] < 200 and
                residuals.max() <= 1.5, "Current image-response fit is not sufficiently constrained/repeatable")
        require(np.linalg.cond(response[:, :2]) < 5 and
                np.linalg.cond(response[:, 3:5]) < 5,
                "Image response does not constrain both lateral and aiming directions")
        require(np.linalg.svd(response[:, :2]*5, compute_uv=False)[-1] >= 2 and
                np.linalg.svd(response[:, 3:5]*2, compute_uv=False)[-1] >= 2,
                "Fresh image-response signal is too weak relative to repeatability limits")
        u, _, vt = np.linalg.svd(response[:, :2])
        self.paper_direction_map = u@vt
        require(np.linalg.det(self.paper_direction_map) > .99, "Measured paper direction response is reflected")
        self.response = response

    def features(self, pose):
        pose = rigid(pose)
        rv = rotation_vector(pose[:3, :3]@self.reference[:3, :3].T)*180/math.pi
        return np.r_[pose[:3, 3]-self.reference[:3, 3], rv[:2]]

    @staticmethod
    def check_repeat(before, after):
        require(after.get("accepted") and after.get("blue_region_on_pattern_px2", 0) < 400,
                "Current response image lost the clear board; stopped without another move")
        a, b = rigid(before["T_base_camera_mount"]), rigid(after["T_base_camera_mount"])
        require(np.linalg.norm(a[:3, 3]-b[:3, 3]) <= .8 and rotation_error(a, b) <= .4,
                "Response acquisition did not return to the same J5 camera pose")
        common = sorted(set(before["corner_ids"]) & set(after["corner_ids"]))
        require(len(common) >= 12, "Too few common corners to verify the stationary board")
        pa = np.asarray(before["corners_px"])[[before["corner_ids"].index(i) for i in common]]
        pb = np.asarray(after["corners_px"])[[after["corner_ids"].index(i) for i in common]]
        require(np.max(np.linalg.norm(pa-pb, axis=1)) <= 5 and
                np.linalg.norm(FramingGuide.measure(before)[1]-FramingGuide.measure(after)[1]) <= 1,
                "Board/image did not repeat at the reference pose; stopped without another move")

    @staticmethod
    def framing_ready(rec, raised=0):
        _, ray = FramingGuide.measure(rec)
        size = np.ptp(np.asarray(rec["framing_guide"]["pattern_outline_px"]), axis=0)/[1920, 1080]
        return (rec["accepted"] and np.linalg.norm(ray-[87.5, 62.5]) <= 10 and
                rec["detection"]["charuco_corner_count"] >= 18 and (size.max() <= 1.02 or raised >= 30))

    @staticmethod
    def measure(rec):
        require("framing_guide" in rec,
                "Board reference lost; stopped without another search move or automatic return")
        guide = rec["framing_guide"]
        obj = np.array([[0, 0], [175, 0], [175, 125], [0, 125]], np.float32)
        h = cv2.getPerspectiveTransform(obj, np.asarray(guide["pattern_outline_px"], np.float32))
        require(np.isfinite(h).all() and abs(np.linalg.det(h)) > 1e-9, "Unstable framing homography")
        centre = np.asarray(guide["image_centre_px"], np.float32).reshape(1, 1, 2)
        ray = cv2.perspectiveTransform(centre, np.linalg.inv(h)).reshape(2)
        require(np.isfinite(ray).all(), "Invalid framing measurement")
        return h, ray

    def prediction(self, rec, target):
        require(self.validated, "Current-pose image response has not passed independent verification")
        h, ray = self.measure(rec)
        df = self.features(target)-self.features(rec["T_base_flange"])
        delta = self.response@df
        height = max(float(rec["T_base_camera_mount"][2][3])-self.setup["table_initial_z_mm"], 150)
        scale = height/(height+df[2])
        require(.7 <= scale <= 1.3, "Framing proposal exceeds the local image-response range")
        centre = np.asarray(rec["framing_guide"]["image_centre_px"])

        def project(obj):
            px = cv2.perspectiveTransform((obj-delta).astype(np.float32).reshape(-1, 1, 2), h).reshape(-1, 2)
            return centre+scale*(px-centre)

        marker_px = project(self.markers.reshape(-1, 2)).reshape(-1, 4, 2)
        visible_markers = np.all((marker_px > [48, 48]) & (marker_px < [1871, 1031]), axis=(1, 2))
        pixels = project(self.obj)
        visible = np.any(visible_markers[self.neighbours], axis=1)
        visible &= np.all((pixels > [48, 48]) & (pixels < [1871, 1031]), axis=1)
        return {"ray": ray+delta, "delta": delta, "pixels": pixels[visible],
                "count": int(sum(visible)), "error": float(np.linalg.norm(ray+delta-[87.5, 62.5]))}

    @staticmethod
    def report(prediction):
        return {"predicted_corner_count": prediction["count"],
                "predicted_image_centre_board_XY_mm": prediction["ray"].tolist(),
                "predicted_centring_error_board_mm": prediction["error"]}

    def bound_target(self, target, planner):
        g = (target@planner.grasp_tcp)[:2, 3]
        if planner.workspace is not None and planner.workspace.get("precalibration_measured_rectangle"):
            # The user authorized the measured rectangle in place of the
            # temporary 35mm disk. Reserve another 5mm for proposal endpoints.
            axes = np.asarray(planner.workspace["axes_base"])[:2, :2]
            origin = np.asarray(planner.workspace["origin_base_mm"])[:2]
            polygon = np.asarray(planner.workspace["polygon_table_xy_mm"])
            xy = axes.T@(g-origin)
            clipped = np.clip(xy, polygon.min(0)+5, polygon.max(0)-5)
            target[:2, 3] += axes@(clipped-xy)
        else:
            require(planner.calibration_anchor_xy is not None, "No confirmed preliminary workspace")
            v = g-planner.calibration_anchor_xy
            if np.linalg.norm(v) > 30:
                target[:2, 3] -= v-v*30/np.linalg.norm(v)
        f = self.features(target)
        require(np.linalg.norm(f[3:5]) <= 20, "Aiming proposal exceeds the measured-response neighbourhood")
        return target

    def pose_for_ray(self, rec, angles, dz, desired_ray, planner):
        target = rigid(rec["T_base_flange"]).copy()
        target[:3, :3] = cv2.Rodrigues(np.r_[angles, 0.0]*math.pi/180)[0]@self.reference[:3, :3]
        target[2, 3] += dz
        _, ray = self.measure(rec)
        df = self.features(target)-self.features(rec["T_base_flange"])
        target[:2, 3] += np.linalg.solve(self.response[:, :2], np.asarray(desired_ray)-ray-self.response@df)
        return self.bound_target(target, planner)

    def verify_response(self, before, after, target, limit_mm=10):
        require(after.get("blue_region_on_pattern_px2", 0) < 400,
                "Blue region obstructs the board; stopped before another motion")
        _, a = self.measure(before)
        _, b = self.measure(after)
        predicted = self.response@(self.features(target)-self.features(before["T_base_flange"]))
        require(0 < limit_mm <= 10, "Invalid image-response verification limit")
        require(np.linalg.norm((b-a)-predicted) <= limit_mm,
                "Fresh image disagrees with the provisional motion response; stopped without another move")


def acquire_current_response(controller, planner, guide, rec, capture, folder):
    """Bounded out-and-back fit probes and a separate current-run check."""
    require(rec["accepted"], "Current response acquisition needs a clear accepted board image: " + rec["reason"])
    origin = rigid(rec["T_base_flange"])
    mount = rigid(rec["T_base_camera_mount"])
    seed = np.asarray(rec["state_after"]["joints"])
    guide.reference = origin.copy()

    def checked_direct(start, goal):
        # Do not let a small probe turn into an RRT detour or an IK branch jump.
        path = [list(map(float, start)), list(map(float, goal))]
        for q in planner.samples(start, goal):
            flange, cm = planner.fk(q), camera_mount_fk(q)
            require(np.max(np.abs(q-seed)) <= 8 and
                    np.linalg.norm(flange[:3, 3]-origin[:3, 3]) <= 8 and
                    rotation_error(flange, origin) <= 2.5 and
                    np.linalg.norm(cm[:3, 3]-mount[:3, 3]) <= 15 and rotation_error(cm, mount) <= 5,
                    "Response probe leaves its bounded neighbourhood; no detour or larger probe allowed")
        planner.preflight(path)
        return path

    probes, plans, attempts = [], [], []
    for axis, amount in enumerate((5, 5, 5, 2, 2)):
        for sign in (1, -1):
            delta = np.zeros(5)
            delta[axis] = sign*amount
            probes.append(("%s_%s" % (("X", "Y", "Z", "Rx", "Ry")[axis],
                                      "plus" if sign > 0 else "minus"), delta))
    def preflight_probe(label, delta):
        target = origin.copy()
        target[:3, 3] += delta[:3]
        target[:3, :3] = cv2.Rodrigues(np.r_[delta[3:5], 0.0]*math.pi/180)[0]@origin[:3, :3]
        plan = {"label": label, "delta_XYZ_mm_RxRy_deg": delta.tolist(), "T_base_flange": target.tolist()}
        try:
            goal = planner.ik(target, seed)
            plan["outward"] = checked_direct(seed, goal)
            plan["return"] = checked_direct(goal, seed)
        except NoIKSolution as exc:
            plan.update(rejected=str(exc), no_ik_solution=True)
        except ValueError as exc:
            plan["rejected"] = str(exc)
        return plan

    log("Preflighting current-VIEW response probes and an independent check; no historical response used.")
    for label, delta in probes:
        plan = preflight_probe(label, delta)
        # The baseline and the required -Z probe still constrain the Z term
        # when +Z has no IK solution. Keep every other fit probe mandatory.
        if label == "Z_plus" and plan.get("no_ik_solution"):
            plan["omitted"] = True
        attempts.append(plan)
        write_json(folder/"response_probe_plan.json", attempts)
        if plan.get("omitted"):
            log("Response probe Z_plus omitted: no IK solution; Z_minus remains required.")
            continue
        require("rejected" not in plan, "Response probe %s preflight rejected: %s" %
                (label, plan.get("rejected", "")))
        plans.append(plan)
    fit_count = len(plans)
    z_probes = ", ".join("%+g" % p["delta_XYZ_mm_RxRy_deg"][2] for p in plans
                         if p["label"].startswith("Z_"))

    # The combined +X/+Z holdout may be unreachable even when each individual
    # axis probe passes. Select from this finite list BEFORE any motion or fit.
    # Every alternative has the same +/-3 mm / +/-1 degree amplitudes and
    # must pass the same IK, direct outward/return and controller/collision gates.
    checks = []
    for delta in ([3, -3, 3, 1, -1], [-3, 3, -3, 1, -1],
                  [-3, -3, -3, -1, 1], [3, 3, -3, -1, 1]):
        plan = preflight_probe("independent_check", np.array(delta, float))
        plan["candidate_index"] = len(checks)
        checks.append(plan)
        write_json(folder/"independent_check_candidates.json", checks)
        if "rejected" not in plan:
            break
    plans.append(plan)
    write_json(folder/"response_probe_plan.json", attempts + [plan])
    require("rejected" not in plan,
            "No independent response check passed preflight within the existing bounds; "
            "no probes moved. See independent_check_candidates.json. " +
            " | ".join("candidate %d: %s" % (c["candidate_index"], c.get("rejected", "passed")) for c in checks))
    selected_delta = plan["delta_XYZ_mm_RxRy_deg"]
    log("Response fit preflight passed: %d probes; base Z offsets %s mm." % (fit_count, z_probes))
    log("Independent response check selected before motion: XYZ %s mm; Rx/Ry %s deg (candidate %d)." %
        (selected_delta[:3], selected_delta[3:], plan["candidate_index"]))
    confirm("MEASURE CURRENT-POSE IMAGE RESPONSE - REAL ROBOT MOTION\n\n"
            "No historical coefficients will be reused. The calibration collector needs a "
            f"fresh response even when the board is already framed. OK authorizes {fit_count} small "
            f"out-and-back fit probes: base X/Y +5 and -5 mm, base Z {z_probes} mm, "
            "world Rx/Ry +2 and -2 degrees, one axis at a time, returning to this pose "
            "after each verified image. One "
            f"independent check uses the preflight-selected XYZ {selected_delta[:3]} mm "
            f"and Rx/Ry {selected_delta[3:]} degrees.\n\n"
            "All direct paths have passed the existing sampled collision/controller checks "
            "inside the temporary 35 mm region. Additional bounds from this pose: flange "
            "8 mm / 2.5 degrees, J5 mount 15 mm / 5 degrees, each joint 8 degrees. "
            "Arm SPEED/ACCEL/DECEL remain 100%; external overrides unchanged. Keep the "
            "board and optics fixed and the swept area clear. Lost board, inconsistent "
            "images or any stop/error ends motion immediately, without a return move. "
            "Cancel performs no response probes.")
    baseline = capture("response_baseline")
    guide.check_repeat(rec, baseline)
    require(np.max(np.abs(np.asarray(baseline["state_after"]["joints"])-seed)) < .05,
            "Robot changed during response confirmation; stopped before probing")
    samples, current_rec = [baseline], baseline
    validation = None

    for index, plan in enumerate(plans):
        independent = index == fit_count
        if independent:
            guide.fit_response(samples)
        state = controller.stable(powered=True, empty=True)
        require(np.max(np.abs(np.asarray(state["joints"])-current_rec["state_after"]["joints"])) < .05,
                "Robot changed since the response image; stopped before another probe")
        planner.sync_check(state)
        path = checked_direct(state["joints"], plan["outward"][-1])
        write_json(folder/("response_%02d_outward.json" % index), {"label": plan["label"], "route": path})
        log("Current response %d/%d: %s; return only after a valid fresh image" %
            (index+1, len(plans), plan["label"]))
        planner.execute(path, "Current response " + plan["label"])
        measured = capture("response_" + plan["label"])
        require(measured["accepted"], "Response probe lost the clear board: " + measured["reason"])
        require(measured["image_sha256"] != current_rec["image_sha256"],
                "Identical camera bytes after a response probe; stopped before returning")
        _, a = guide.measure(current_rec)
        _, b = guide.measure(measured)
        require(np.linalg.norm(b-a) <= 25,
                "Small response probe caused excessive image displacement; stopped before returning")
        if plan["label"].startswith(("X_", "Y_", "Rx_", "Ry_")):
            require(np.linalg.norm(b-a) >= 1,
                    "Response probe produced too little image change; stopped before returning")
        actual = rigid(measured["T_base_flange"])
        target = rigid(plan["T_base_flange"])
        require(np.linalg.norm(actual[:3, 3]-target[:3, 3]) <= planner.setup["planning"]["position_tolerance_mm"] and
                rotation_error(actual, target) <= planner.setup["planning"]["angle_tolerance_deg"],
                "Response probe did not reach its requested physical-flange pose")
        if independent:
            predicted = guide.response@(guide.features(actual)-guide.features(current_rec["T_base_flange"]))
            error = float(np.linalg.norm((b-a)-predicted))
            validation = {"before_image": current_rec["image_file"], "check_image": measured["image_file"],
                          "preflight_candidate_index": plan["candidate_index"],
                          "planned_delta_XYZ_mm_RxRy_deg": plan["delta_XYZ_mm_RxRy_deg"],
                          "not_used_for_fit": True, "predicted_delta_board_mm": predicted.tolist(),
                          "measured_delta_board_mm": (b-a).tolist(), "error_board_mm": error,
                          "limit_board_mm": 2.0, "passed": False}
            write_json(folder/"response_validation.json", validation)
            require(error <= 2, "Independent current-response check failed (%.2f board mm); no return or collection" % error)
        else:
            samples.append(measured)
        state = controller.stable(powered=True, empty=True)
        require(np.max(np.abs(np.asarray(state["joints"])-measured["state_after"]["joints"])) < .05,
                "Robot changed after the response image; stopped before returning")
        back = checked_direct(state["joints"], seed)
        write_json(folder/("response_%02d_return.json" % index), {"route": back})
        planner.execute(back, "Verified response return " + plan["label"])
        current_rec = capture("response_return_%02d" % index)
        guide.check_repeat(baseline, current_rec)
    require(validation is not None,
            "Independent response check did not run; no collection or return was started")
    validation.update({"passed": True, "return_image": current_rec["image_file"],
                       "base_XY_to_current_board_direction_map": guide.paper_direction_map.tolist()})
    write_json(folder/"response_validation.json", validation)
    guide.validated = True
    log("Current-pose response verified: independent error %.2f board mm; reference image repeated. "
        "This is motion guidance, NOT accepted camera calibration." % validation["error_board_mm"])
    return current_rec


def apply_measured_paper_workspace(controller, planner, guide, rec, setup, folder):
    """Use the fixed paper direction, checked against this run's measured response."""
    limits = setup.get("reported_paper_limits_mm", {})
    keys = ("left", "right", "behind", "toward_user")
    require(all(k in limits and math.isfinite(float(limits[k])) and float(limits[k]) > 10 for k in keys),
            "Measured paper limits are missing; no workspace expansion is allowed")
    frame = cv2.imread(str(folder/rec["image_file"]))
    require(frame is not None, "Fresh paper-direction snapshot could not be reopened")
    h, ray = guide.measure(rec)
    direction = np.asarray(FIXED_CALIBRATION["toward_user_board_xy"], float)
    require(direction.shape == (2,) and direction[0] == 0 and abs(direction[1]) == 1,
            "Fixed paper direction must follow the signed long axis of the print")
    forward = guide.paper_direction_map.T@direction
    forward /= np.linalg.norm(forward)
    expected = array(FIXED_CALIBRATION["expected_toward_user_base_xy"], (2,), "fixed paper direction")
    require(np.isclose(np.linalg.norm(expected), 1) and forward@expected >=
            math.cos(math.radians(FIXED_CALIBRATION["paper_direction_tolerance_deg"])),
            "Fresh paper direction differs from the fixed setup; no rectangle was applied")
    right = np.array([-forward[1], forward[0]])
    right_board = guide.paper_direction_map@right
    arrow_board = np.array([ray, ray+30*direction, ray+30*right_board], np.float32)
    arrow_px = cv2.perspectiveTransform(arrow_board.reshape(-1, 1, 2), h).reshape(-1, 2)
    require(np.isfinite(arrow_px).all() and np.max(np.abs(arrow_px)) < 100000, "Paper-direction projection invalid")
    drawn = frame.copy()
    for i, label, colour in ((1, "TOWARDS YOU", (0, 255, 255)), (2, "YOUR RIGHT", (255, 255, 0))):
        a, b = np.rint(arrow_px[[0, i]]).astype(int)
        cv2.arrowedLine(drawn, tuple(a), tuple(b), colour, 3, tipLength=.1)
        at = np.clip(b+[10, -10], [5, 90], [1550, 1050]).astype(int)
        cv2.putText(drawn, label, tuple(at), cv2.FONT_HERSHEY_SIMPLEX, .7, colour, 2)
    show_image(drawn, "FIXED TABLE DIRECTIONS: yellow TOWARDS YOU; cyan YOUR RIGHT")
    cv2.imwrite(str(folder/"paper_direction_confirmation.png"), drawn)
    current = controller.stable(powered=True, empty=True)
    require(np.max(np.abs(np.asarray(current["joints"])-rec["state_after"]["joints"])) < .05,
            "Robot moved while orienting the paper; no rectangle was applied")
    anchor = planner.calibration_anchor_xy.copy()
    r3, f3 = np.r_[right, 0.0], np.r_[forward, 0.0]
    axes = np.column_stack([r3, f3, np.cross(f3, r3)])
    left, right_limit, behind, front = [float(limits[k]) for k in keys]
    polygon = [[-left+5, -behind+5], [right_limit-5, -behind+5],
               [right_limit-5, front-5], [-left+5, front-5]]
    workspace = {"method": "fixed measured pre-calibration rectangle",
                 "precalibration_measured_rectangle": True,
                 "axes_base": axes.tolist(), "origin_base_mm": np.r_[anchor, setup["table_initial_z_mm"]].tolist(),
                 "polygon_table_xy_mm": polygon, "measured_limits_mm": {k: float(limits[k]) for k in keys},
                 "boundary_inset_mm": 5.0,
                 "direction_source": "fixed board axis and current-run verified robot/image response",
                 "fixed_setup_verified": True}
    planner.workspace = workspace
    planner.calibration_anchor_xy = None
    planner.cache.clear()
    planner.sync_check(current)
    require(planner.valid(current["joints"]), "Current pose fails the confirmed paper/collision checks")
    write_json(folder/"precalibration_workspace.json", workspace)
    log("Fixed measured paper rectangle active (5 mm inset). Towards-you base XY: %s; your-right base XY: %s" %
        (np.round(forward, 4).tolist(), np.round(right, 4).tolist()))


def acquire_calibration_board(camera, controller, planner, cfg, setup, folder):
    attempts = []

    def capture(label):
        controller.stable(powered=True, empty=True)
        rec = camera.observation(controller, label, "acquisition")
        attempts.append(rec)
        write_json(folder/"anchor_attempts.json", attempts)
        log("Board framing: " + rec["reason"])
        return rec

    rec = capture("anchor_view")
    require(rec.get("blue_region_on_pattern_px2", 0) < 400,
            "A blue region covers the pattern. Remove the obstruction before starting a new calibration.")
    require(rec["accepted"], "Automatic Set View needs a clear board at saved VIEW: " + rec["reason"])
    view_check(rec["state_after"], cfg)
    # Stage only the canonical VIEW. The live readback and image are recorded;
    # the active VIEW/calibration binding is committed only after acceptance.
    write_json(folder/"new_view.json", {"observation": cfg["observation"],
               "state": rec["state_after"], "image_file": rec["image_file"], "staged": True})
    log("Automatic Set View verified the saved VIEW; no configuration written.")
    guide = FramingGuide(camera, setup, folder)
    guide.measure(rec)  # A usable marker reference is mandatory before any move.
    log("VIEW framing: %s; current-pose response is still required for collection." %
        ("already adequate; no corrective framing needed" if guide.framing_ready(rec) else
         "correction needed under the existing centring/visibility requirements"))
    rec = acquire_current_response(controller, planner, guide, rec, capture, folder)
    apply_measured_paper_workspace(controller, planner, guide, rec, setup, folder)
    if guide.framing_ready(rec):
        log("Current image meets framing requirements; skipping all corrective framing moves.")
        return rec, guide
    confirm("IMAGE-DIRECTED BOARD FRAMING\n\nOK authorizes checked corrections based on "
            "the fresh marker image and the independently checked response measured in this run. "
            "Lateral corrections are tried first (two verified 5 mm steps, then up to 10 mm), with small "
            "aiming corrections (up to 2 degrees) if needed. Height is increased only "
            "after centring if the pattern still fills too much of the image. The old "
            "upward-first sweep is removed.\n\n"
            "Command speed/acceleration/deceleration remain 100%. Existing joint limits, "
            "collision checks and your confirmed measured paper rectangle remain. Keep the clear board fixed. "
            "Every correction is verified in a fresh image; lost/inconsistent reference stops.")
    # A separate framing confirmation can take time. Refresh before moving.
    ready = capture("anchor_ready")
    guide.check_repeat(rec, ready)
    require(np.max(np.abs(np.asarray(ready["state_after"]["joints"])-rec["state_after"]["joints"])) < .05,
            "Robot moved during framing confirmation; stopped before a correction")
    _, old_ray = guide.measure(rec)
    _, ready_ray = guide.measure(ready)
    require(np.linalg.norm(ready_ray-old_ray) <= 2,
            "Board changed during direction/framing confirmation; stopped before a correction")
    rec = ready
    view = controller.physical_pose(cfg["observation"]["pose"])
    stalled = 0
    for step in range(18):
        _, ray = guide.measure(rec)
        error = np.array([87.5, 62.5])-ray
        size = np.ptp(np.asarray(rec["framing_guide"]["pattern_outline_px"]), axis=0)/[1920, 1080]
        raised = float(rec["T_base_flange"][2][3])-view[2, 3]
        if guide.framing_ready(rec, raised):
            return rec, guide
        current = controller.stable(powered=True, empty=True)
        require(np.max(np.abs(np.asarray(current["joints"])-rec["state_after"]["joints"])) < .05,
                "Robot changed since the framing image; stopped before another correction")
        planner.sync_check(current)
        original = rigid(rec["T_base_flange"])
        xy = np.linalg.solve(guide.response[:, :2], error)
        xy *= min(1.0, (5 if step < 2 else 10)/max(np.linalg.norm(xy), 1e-9))
        tilt = np.linalg.solve(guide.response[:, 3:5], error)
        tilt *= min(1.0, 2/max(np.linalg.norm(tilt), 1e-9))
        lateral = original.copy()
        lateral[:2, 3] += xy
        aim = original.copy()
        angles = guide.features(original)[3:5]+tilt
        aim[:3, :3] = cv2.Rodrigues(np.r_[angles, 0.0]*math.pi/180)[0]@guide.reference[:3, :3]
        options = [("lateral image correction", lateral), ("small camera aiming correction", aim)]
        # Height is only needed when the actual framing size gate fails after
        # centring. All corners visible with size <=1.02 needs no height move.
        if np.linalg.norm(error) <= 10 and size.max() > 1.02 and raised < 40:
            higher = original.copy()
            higher[2, 3] += min(15, 40-raised)
            options.insert(0, ("centred pattern needs wider field; limited height correction", higher))
        chosen = None
        rejections = []
        for reason, target in options:
            try:
                target = guide.bound_target(target, planner)
                prediction = guide.prediction(rec, target)
                if "height" not in reason and prediction["error"] >= np.linalg.norm(error)-1:
                    continue
                goal = planner.ik(target, np.asarray(current["joints"]))
                route = planner.route(current["joints"], goal)
                planner.preflight(route)
                chosen = reason, target, prediction, route
                break
            except ValueError as exc:
                rejections.append(reason + ": " + str(exc))
                write_json(folder/("framing_%02d_%s_rejected.json" % (step, reason.split()[0])),
                           {"reason": str(exc), "T_base_flange": target.tolist(),
                            "start_joints": current["joints"]})
        require(chosen is not None, "No image-improving checked correction fits the existing "
                "workspace/reach limits. Stopped; no blind sweep or limit change. " + " | ".join(rejections))
        reason, target, prediction, route = chosen
        delta = target[:3, 3]-original[:3, 3]
        aim_delta = guide.features(target)[3:5]-guide.features(original)[3:5]
        log("Framing %d: %s; base XYZ delta %s mm; aiming Rx/Ry delta %s deg; "
            "predicted centring error %.1f -> %.1f board mm" %
            (step+1, reason, np.round(delta, 2).tolist(), np.round(aim_delta, 2).tolist(),
             np.linalg.norm(error), prediction["error"]))
        write_json(folder/("framing_%02d_route.json" % step),
                   {"reason": reason, "T_base_flange": target.tolist(), "route": route,
                    "base_delta_mm": delta.tolist(), "world_aim_delta_deg": aim_delta.tolist(),
                    "image_prediction": guide.report(prediction)})
        planner.execute(route, "Image-directed framing %d" % (step+1))
        newer = capture("anchor_guided_%02d" % step)
        guide.verify_response(rec, newer, target)
        _, new_ray = guide.measure(newer)
        new_error = np.linalg.norm(new_ray-[87.5, 62.5])
        require("height" in reason or new_error <= np.linalg.norm(error)+3,
                "Centring correction made the image worse; stopped without another move")
        stalled = stalled+1 if "height" not in reason and new_error >= np.linalg.norm(error)-.5 else 0
        require(stalled < 2, "Image centring is not improving; stopped without continuing a sweep")
        rec = newer
    raise ValueError("Image-directed framing reached its correction limit. No calibration "
                     "collection or blind fallback started; see the fresh image and route records.")


def capture_bounded_collection_route(camera, controller, planner, guide, rec, route, label, role, folder):
    """Verify fresh images along the route; transit images are not fit/check data."""
    kinematic_cache = {}

    def kinematics(q):
        # Pure model geometry only; exact keys, scoped to chunk construction or
        # one segment. No controller readings, range approvals or checks cached.
        key = tuple(map(float, q))
        if key not in kinematic_cache:
            kinematic_cache[key] = planner.fk(q), camera_mount_fk(q)
        return kinematic_cache[key]

    def within(a, b, mm, deg):
        fa, ma = kinematics(a)
        fb, mb = kinematics(b)
        return (np.max(np.abs(np.asarray(a)-b)) <= deg and
                np.linalg.norm(fa[:3, 3]-fb[:3, 3]) <= mm and rotation_error(fa, fb) <= deg and
                np.linalg.norm(ma[:3, 3]-mb[:3, 3]) <= mm and rotation_error(ma, mb) <= deg)

    chunks = []
    for a, b in zip(route, route[1:]):
        dense = planner.samples(a, b)
        first = last = 0
        # Preserve every original route edge; never shortcut a checked detour.
        for end in range(1, len(dense)):
            if not within(dense[first], dense[end], 7, 1.5):
                require(last > first, "Route sampling cannot satisfy bounded collection steps")
                chunks.append(dense[last].tolist())
                first = last
                require(within(dense[first], dense[end], 7, 1.5),
                        "Route sample exceeds bounded collection step; stopped before motion")
            last = end
        if last > first:
            chunks.append(dense[last].tolist())
    if not chunks:
        chunks = [list(map(float, route[-1]))]
    write_json(folder/(label+"_bounded_route.json"),
               {"original_route": route, "image_checkpoint_joints": chunks,
                "max_flange_and_J5_translation_mm": 8, "max_flange_and_J5_rotation_deg": 2,
                "max_joint_change_deg": 2, "max_response_error_board_mm": 2})
    observations = []
    for step, goal in enumerate(chunks):
        checkpoint_started = time.monotonic()
        kinematic_cache.clear()  # Fresh geometry for the actual segment start.
        check_stop()
        state = controller.stable(powered=True, empty=True)
        require(np.max(np.abs(np.asarray(state["joints"])-rec["state_after"]["joints"])) < .05,
                "Robot changed since the collection checkpoint image; discard remaining route")
        planner.sync_check(state)
        path = [state["joints"], goal]
        require(all(within(state["joints"], q, 8, 2) for q in planner.samples(state["joints"], goal)),
                "Collection segment exceeds 8 mm / 2 degree bounds; stopped before motion")
        target = planner.fk(goal)
        prediction = guide.prediction(rec, target)
        require(prediction["count"] >= 14,
                "Next bounded collection segment lacks predicted board visibility; stopped before motion")
        final = step == len(chunks)-1
        shot_label = label if final else label+"_transit_%02d" % step
        shot_role = role if final else "acquisition"
        log("Bounded collection %s: segment %d/%d; <=8 mm / 2 deg; fresh image follows" %
            (label, step+1, len(chunks)))
        after_move = planner.execute(path, "Bounded " + shot_label)
        observation_started = time.monotonic()
        newer = camera.observation(controller, shot_label, shot_role, before_state=after_move)
        observation_seconds = time.monotonic()-observation_started
        observations.append(newer)
        write_json(folder/(label+"_checkpoint_observations.json"), observations)
        require(newer["accepted"], "Board lost at a collection checkpoint: " + newer["reason"])
        moved = np.max(np.abs(np.asarray(state["joints"])-goal)) > .01
        require(not moved or newer["image_sha256"] != rec["image_sha256"],
                "Identical camera bytes after collection movement; stopped without another move")
        _, a = guide.measure(rec)
        _, b = guide.measure(newer)
        actual_target = rigid(newer["T_base_flange"])
        predicted_delta = guide.response@(guide.features(actual_target)-guide.features(rec["T_base_flange"]))
        error = float(np.linalg.norm((b-a)-predicted_delta))
        write_json(folder/(shot_label+"_response_check.json"),
                   {"before_image": rec["image_file"], "after_image": newer["image_file"],
                    "target_T_base_flange": target.tolist(), "route": path,
                    "prediction": guide.report(prediction), "predicted_delta_board_mm": predicted_delta.tolist(),
                    "measured_delta_board_mm": (b-a).tolist(), "error_board_mm": error,
                    "limit_board_mm": 2, "passed": error <= 2,
                    "checkpoint_seconds": time.monotonic()-checkpoint_started,
                    "observation_seconds": observation_seconds})
        # The original 10 mm framing gate remains. These small collection steps
        # use a stricter 2 mm gate, also used by the independent response probe.
        guide.verify_response(rec, newer, actual_target, limit_mm=2)
        # Refresh the local derivative only AFTER independent agreement with
        # the previous estimate. The following image checks this update; it is
        # never fitted to an image first and then used to approve that image.
        scales = np.array([5, 5, 5, 2, 2], float)
        df = (guide.features(actual_target)-guide.features(rec["T_base_flange"]))/scales
        if np.linalg.norm(df) >= .25:
            old_response = guide.response*scales
            change = np.outer((b-a)-predicted_delta, df)/np.dot(df, df)
            updated = (old_response+change)/scales
            require(np.linalg.norm(change) <= .25*np.linalg.norm(old_response) and
                    np.linalg.cond(updated[:, :2]) < 5 and np.linalg.cond(updated[:, 3:5]) < 5 and
                    np.linalg.det(updated[:, :2]) > 0,
                    "Local response update is unstable; stopped without another move")
            write_json(folder/(shot_label+"_local_response.json"),
                       {"before_image": rec["image_file"], "after_image": newer["image_file"],
                        "response_before": guide.response.tolist(), "response_after": updated.tolist(),
                        "purpose": "local guidance only; next image independently checks this update"})
            guide.response = updated
        rec = newer
    return rec


def collect_guided_views(camera, controller, planner, anchor, guide, setup, folder):
    """Prepare visible/reachable image-directed targets before collecting data."""
    require(guide.validated, "Collection needs an independently verified response from this run")
    current = controller.stable(powered=True, empty=True)
    require(np.max(np.abs(np.asarray(current["joints"])-anchor["state_after"]["joints"])) < .05,
            "Robot changed since the verified anchor image; stopped before collection")
    base_angles = guide.features(anchor["T_base_flange"])[3:5]
    specs = []
    for amplitude in (4, 8, 12, 16, 20):
        for rx, ry in ((amplitude, 0), (-amplitude, 0), (0, amplitude), (0, -amplitude),
                       (amplitude, amplitude/2), (-amplitude, -amplitude/2),
                       (amplitude/2, -amplitude), (-amplitude/2, amplitude)):
            for dz in (-30, -15, 0, 15):
                specs.append((base_angles+[rx, ry], dz, np.array([87.5, 62.5])))
    # The 30 October real pool reached only 82.6% X span with +/-45 mm.
    # Sample +/-50 too, under the SAME visibility/IK/workspace/route gates.
    for dx, dy in ((-45, 0), (45, 0), (0, -30), (0, 30),
                   (-35, -25), (-35, 25), (35, -25), (35, 25), (-50, 0), (50, 0)):
        for dz in (-30, -15, 0, 15):
            specs.append((base_angles, dz, np.array([87.5+dx, 62.5+dy])))
    # The 4 November VIEW reached 83.5% X span. Give the pool wider image-ray
    # offsets at the two lower heights where both existing +/-50 targets passed.
    for dx in (-55, 55, -60, 60):
        for dz in (-30, -15):
            specs.append((base_angles, dz, np.array([87.5+dx, 62.5])))
    # The calibrated image hull is the convex hull of observed corners. The
    # right/top border was under-covered and blocked edge picks, so add
    # periphery aims (20 deg is the measured-response limit) and vertical and
    # diagonal image-centre offsets. Every extra spec passes the same
    # visibility/IK/workspace/route gates and the standard collection checks;
    # no quality threshold or limit is relaxed.
    for dx, dy in ((0, -45), (0, 45), (0, -60), (0, 60),
                   (-35, -45), (-35, 45), (35, -45), (35, 45),
                   (-60, -45), (-60, 45)):
        for dz in (-30, -15, 0):
            specs.append((base_angles, dz, np.array([87.5+dx, 62.5+dy])))
    pool, decisions = [], []
    for index, (angles, dz, desired_ray) in enumerate(specs):
        check_stop()
        try:
            target = guide.pose_for_ray(anchor, angles, dz, desired_ray, planner)
            prediction = guide.prediction(anchor, target)
            require(prediction["count"] >= 14, "Predicted complete-marker visibility is insufficient")
            goal = planner.ik(target, np.asarray(current["joints"]))
            require(planner.valid(goal), "Target violates the existing workspace/collision checks")
            require(all(np.linalg.norm(goal-p["joints"]) >= 1.5 for p in pool), "Duplicate candidate")
            pool.append({"id": index, "target": target, "joints": goal,
                         "mount": camera_mount_fk(goal), "predicted_pixels": prediction["pixels"]})
            decisions.append({"id": index, "T_base_flange": target.tolist(),
                              "joints": goal.tolist(), "prediction": guide.report(prediction)})
        except ValueError as exc:
            decisions.append({"id": index, "rejected": str(exc)})
    write_json(folder/"guided_candidate_pool.json", decisions)
    quality = setup["quality"]
    needed = max(quality["min_fit"]+quality["min_check"], 4*quality["min_check"],
                 4*math.ceil(quality["min_fit"]/3))
    require(len(pool) >= needed,
            "Only %d distinct, predicted-visible reachable poses fit the current limits; "
            "%d are needed for fit/check allocation. No calibration sweep started. "
            "See guided_candidate_pool.json; no workspace or joint limits were widened." % (len(pool), needed))
    all_pixels = np.concatenate([p["predicted_pixels"] for p in pool]).astype(np.float32)
    possible_span = np.ptp(all_pixels, axis=0)/[1920, 1080]
    possible_coverage = float(cv2.contourArea(cv2.convexHull(all_pixels))/(1920*1080))
    write_json(folder/"guided_pool_coverage.json", {"span_upper_bound_xy": possible_span.tolist(),
               "coverage_upper_bound": possible_coverage, "reachable_pose_count": len(pool),
               "pixel_min_xy": all_pixels.min(0).tolist(), "pixel_max_xy": all_pixels.max(0).tolist(),
               "required_span_xy": quality["coverage_span_xy"],
               "required_coverage": quality["coverage_fraction"]})
    require(np.all(possible_span >= quality["coverage_span_xy"]) and
            possible_coverage >= quality["coverage_fraction"],
            "Prepared poses cannot cover enough of the image (predicted spans %s, area %.1f%%). "
            "No collection started; see guided_pool_coverage.json. Quality limits remain unchanged." %
            (np.round(possible_span, 3).tolist(), 100*possible_coverage))
    largest_pair = max(rotation_error(a["mount"], b["mount"]) for a, b in itertools.combinations(pool, 2))
    require(largest_pair >= 15,
            "Prepared visible poses lack 15 degrees of rotation variation; no calibration sweep started")
    # The fit measures board tilt from the first accepted view, not rotation in
    # general. Roll about the optical axis passes the checks above but leaves
    # the intrinsics as weakly constrained as translation, so require a pose
    # that actually tilts the board relative to the saved VIEW.
    anchor_mount = np.asarray(anchor["T_base_camera_mount"], float)
    reachable_tilt = max(viewing_tilt(anchor_mount, item["mount"]) for item in pool)
    require(reachable_tilt >= 15,
            "Prepared visible poses lack 15 degrees of board tilt from the saved VIEW; "
            "intrinsics would be weakly constrained. No calibration sweep started")
    rotations = np.array([rotation_vector(a["mount"][:3, :3].T@b["mount"][:3, :3])
                          for a, b in itertools.combinations(pool, 2)])
    singular = np.linalg.svd(rotations, compute_uv=False)
    require(singular[1] >= .4 and singular[1]/singular[0] >= .15,
            "Prepared visible poses lack two-axis rotation excitation; no calibration sweep started")
    log("Prepared %d predicted-visible reachable poses. Each selected route will be checked before motion." % len(pool))
    records, accepted, rec = [anchor], [], anchor
    complete = False
    collection_started = time.monotonic()
    for shot in range(min(len(pool), 40)):
        check_stop()
        current = controller.stable(powered=True, empty=True)
        require(np.max(np.abs(np.asarray(current["joints"])-rec["state_after"]["joints"])) < .05,
                "Robot changed since the last calibration image; discard the remaining proposals")
        planner.sync_check(current)
        # Role is fixed before inspecting this new image.
        role = "check" if shot % 4 == 3 else "fit"
        train = [r for r in records if r.get("accepted") and r["role"] == "fit"]
        old_pixels = np.concatenate([np.asarray(r["corners_px"], np.float32) for r in train]) if train else None
        old_area = cv2.contourArea(cv2.convexHull(old_pixels)) if old_pixels is not None else 0
        old_span = np.ptp(old_pixels, axis=0)/[1920, 1080] if old_pixels is not None else np.zeros(2)
        train_mounts = [np.asarray(r["T_base_camera_mount"], float) for r in train]
        max_tilt = max((viewing_tilt(anchor_mount, h) for h in train_mounts), default=0)
        covered = bool(train) and (
            np.all(np.ptp(old_pixels, axis=0)/[1920, 1080] >= quality["coverage_span_xy"]) and
            old_area/(1920*1080) >= quality["coverage_fraction"])
        ranked = []
        for item in pool:
            prediction = guide.prediction(rec, item["target"])
            if prediction["count"] < 14:
                continue
            if accepted:
                angular = min(rotation_error(item["mount"], r["mount"]) for r in accepted)
                distance = min(np.linalg.norm(item["mount"][:3, 3]-r["mount"][:3, 3]) for r in accepted)
            else:
                # A fresh clear anchor is already available. Do not jump to an
                # angular extreme to establish the first fit reference.
                angular = distance = 0
            px = prediction["pixels"].astype(np.float32)
            combined = px if old_pixels is None else np.vstack([old_pixels, px])
            gain = (cv2.contourArea(cv2.convexHull(combined))-old_area)/(1920*1080)
            score = 4*angular+.3*min(distance, 30)+150*gain+.2*prediction["count"]
            if role == "fit":
                new_span = np.ptp(combined, axis=0)/[1920, 1080]
                span_gain = np.sum(np.minimum(new_span, quality["coverage_span_xy"])-
                                   np.minimum(old_span, quality["coverage_span_xy"]))
                score += 600*float(span_gain)  # Fill missing sensor regions before redundant centred views.
            travel = np.linalg.norm(item["target"][:3, 3]-np.asarray(rec["T_base_flange"])[:3, 3])
            turn = rotation_error(item["mount"], rigid(rec["T_base_camera_mount"]))
            score -= .8*travel+3*turn  # Prefer nearby useful poses over opposite extremes.
            tilt = viewing_tilt(item["mount"], anchor_mount)
            if role == "fit" and covered and max_tilt < 15:
                # The fit gate needs >=15 deg of out-of-plane board tilt. Once the
                # image-coverage targets are met, prefer poses that supply it
                # instead of more redundant centred views.
                score += 12*tilt
            ranked.append((score, item, prediction))
        ranked.sort(key=lambda x: x[0], reverse=True)
        chosen = None
        for _, item, prediction in ranked:
            try:
                route = planner.route(current["joints"], item["joints"])
                planner.preflight(route)
                chosen = item, prediction, route
                break
            except ValueError as exc:
                write_json(folder/("guided_%03d_route_rejected.json" % item["id"]), {"reason": str(exc)})
                pool = [p for p in pool if p["id"] != item["id"]]
        require(chosen is not None, "No further predicted-visible checked calibration route remains; "
                "stopped without trying unseen/off-image poses")
        item, prediction, route = chosen
        pool = [p for p in pool if p["id"] != item["id"]]
        delta = item["target"][:3, 3]-np.asarray(rec["T_base_flange"])[:3, 3]
        log("Guided %s %d: base XYZ delta %s mm; predicted %d corners; selecting pose variation/image coverage" %
            (role, shot+1, np.round(delta, 2).tolist(), prediction["count"]))
        write_json(folder/("guided_%02d_route.json" % shot),
                   {"role": role, "candidate_id": item["id"], "route": route,
                    "T_base_flange": item["target"].tolist(), "prediction": guide.report(prediction)})
        newer = capture_bounded_collection_route(camera, controller, planner, guide, rec, route,
                                                 "guided_%02d" % shot, role, folder)
        records.append(newer)
        write_json(folder/"observations.json", records)
        require(newer["accepted"], "A guided calibration view did not retain enough board corners: "
                + newer["reason"] + ". Stopped before another move; inspect this prediction/image pair.")
        accepted.append(item)
        rec = newer
        train = [r for r in records if r.get("accepted") and r["role"] == "fit"]
        checks = [r for r in records if r.get("accepted") and r["role"] == "check"]
        pix = np.concatenate([np.asarray(r["corners_px"], np.float32) for r in train])
        span = np.ptp(pix, axis=0)/[1920, 1080]
        coverage = cv2.contourArea(cv2.convexHull(pix))/(1920*1080)
        mounts = [np.asarray(r["T_base_camera_mount"]) for r in train]
        translation = float(np.linalg.norm(np.ptp([h[:3, 3] for h in mounts], axis=0)))
        angular = max((rotation_error(mounts[0], h) for h in mounts), default=0)
        tilt = max((viewing_tilt(anchor_mount, h) for h in mounts), default=0)
        rotations = np.array([rotation_vector(a[:3, :3].T@b[:3, :3])
                              for a, b in itertools.combinations(mounts, 2)])
        singular = np.linalg.svd(rotations, compute_uv=False) if len(rotations) else np.zeros(3)
        excitation = len(singular) >= 2 and singular[0] > 0 and singular[1] >= .4 and singular[1]/singular[0] >= .15
        progress = {"fit": len(train), "check": len(checks), "training_span_xy": span.tolist(),
                    "required_fit": quality["min_fit"], "required_check": quality["min_check"],
                    "required_span_xy": quality["coverage_span_xy"],
                    "required_coverage": quality["coverage_fraction"],
                    "collection_elapsed_seconds": time.monotonic()-collection_started,
                    "training_coverage": float(coverage), "mount_translation_span_mm": translation,
                    "mount_rotation_from_first_fit_deg": angular,
                    "mount_tilt_from_first_fit_deg": tilt,
                    "mount_rotation_excitation_singular_values": singular.tolist()}
        write_json(folder/"guided_collection_progress.json", progress)
        log("Guided collection: fit %d/%d; check %d/%d; image spans %s (need %s); "
            "sensor-area coverage %.1f%% (need %.1f%%; NOT completion); board tilt %.1f deg "
            "(need 15.0); elapsed %.1f min" %
            (len(train), quality["min_fit"], len(checks), quality["min_check"],
             np.round(span, 3).tolist(), quality["coverage_span_xy"], 100*coverage,
             100*quality["coverage_fraction"], tilt, (time.monotonic()-collection_started)/60))
        complete = (len(train) >= quality["min_fit"] and len(checks) >= quality["min_check"] and
                    np.all(span >= quality["coverage_span_xy"]) and coverage >= quality["coverage_fraction"] and
                    translation >= 25 and angular >= 15 and tilt >= 15 and excitation)
        if complete:
            break
    require(complete, "Checked visible-pose collection did not reach count/coverage/tilt/excitation needs. "
            "Stopped; see guided_collection_progress.json. No quality thresholds were relaxed.")
    return records


def confirm_previous_workspace_return(camera, controller, planner, cfg, setup, folder):
    """A new, confirmed startup return after stopping outside the 35 mm disk."""
    state = controller.stable(powered=True, empty=True)
    anchor = planner.calibration_anchor_xy.copy()
    current_xy = (planner.fk(state["joints"])@planner.grasp_tcp)[:2, 3]
    if np.linalg.norm(current_xy-anchor) <= 35:
        return None
    source = None
    for candidate in sorted((ROOT/"sessions").iterdir(), reverse=True):
        if candidate == Path(folder) or not (candidate/"precalibration_workspace.json").is_file():
            continue
        # Only consider the most recent confirmed rectangle. Never fall back
        # through older boundaries until a permissive one happens to fit.
        source = candidate
        break
    require(source is not None, "Outside startup region with no previous confirmed rectangle; stopped before return")
    workspace = read_json(source/"precalibration_workspace.json")
    previous = read_json(source/"start_state.json")
    attempts = read_json(source/"anchor_attempts.json")
    require(workspace.get("precalibration_measured_rectangle") and
            (workspace.get("operator_confirmed") or workspace.get("fixed_setup_verified")) and
            previous["setup"] == setup and previous["frames"] == controller.frames(),
            "Previous rectangle setup/frames do not match; no startup return authorized")
    view_check(attempts[0]["state_after"], cfg)
    require(np.linalg.norm(np.asarray(workspace["origin_base_mm"])[:2]-anchor) <= .8,
            "Previous rectangle is anchored to a different saved VIEW")
    camera.frame("startup_return_confirmation", controller)
    confirm("NEW STARTUP RETURN USING YOUR PREVIOUSLY CONFIRMED PAPER RECTANGLE\n\n"
            "The current pose is outside the temporary 35 mm circle. A prior confirmed "
            "rectangle matches this saved VIEW, setup and controller frames: " + source.name + ".\n\n"
            "Confirm the paper/table boundary and its directions have NOT moved, the "
            "measurements remain left70/right140/behind40/towards240 mm, and the swept area "
            "is clear. OK authorizes a newly planned collision/controller-checked return "
            "to the existing VIEW within that same rectangle, at SPEED/ACCEL/DECEL=100 "
            "with external overrides unchanged. The temporary 35 mm probe region is "
            "restored at VIEW. No failed collection route or old response is resumed. "
            "Cancel if anything changed or is uncertain; no return is performed.")
    current = controller.stable(powered=True, empty=True)
    require(np.max(np.abs(np.asarray(current["joints"])-state["joints"])) < .05,
            "Robot moved during startup return confirmation")
    planner.workspace = workspace
    planner.calibration_anchor_xy = None
    planner.cache.clear()
    require(planner.valid(current["joints"]), "Current pose fails the previously confirmed rectangle/collision checks")
    write_json(folder/"startup_return_workspace.json", {"source_session": source.name,
               "workspace": workspace, "operator_reconfirmed": True, "state": current})
    return anchor


def ensure_gripper_open(controller, setup, folder):
    """Return a stationary state with an open, empty gripper.

    A hand that is already open and empty costs no dialog and no command.
    Otherwise the operator is told to remove any object and must approve one
    open command, so nothing is released without an explicit confirmation.
    """
    hand = setup["gripper"]
    open_mm = float(hand["open_position_mm"])
    state = controller.stable()
    if not state["hand_holding"] and abs(state["hand_position"] - open_mm) <= .4:
        return state
    require(state["servo"],
            "Gripper is not fully open and motor power is OFF. Open it with the pendant, "
            "or enable power there, then run again; this program does not enable power "
            "to release it.")
    confirm("GRIPPER IS NOT FULLY OPEN (%.1f mm%s).\n\n"
            "Remove any object from the gripper by hand now. OK then opens the gripper "
            "at this pose to %.1f mm; anything still held will be released. Keep hands "
            "clear of the fingers and of the area below them. Cancel stops without "
            "moving the gripper." % (state["hand_position"],
                                     ", holding" if state["hand_holding"] else "", open_mm))
    controller.take()
    before = controller.stable(powered=True)
    if not before["hand_holding"] and abs(before["hand_position"] - open_mm) <= .4:
        return before
    write_json(folder/"startup_gripper_open.json", {"stage": "opening", "before": before})
    controller.check_frames()
    controller.check_motion_state()
    log("Gripper is not open: opening to %.1f mm at the current pose." % open_mm)
    controller.hand("HandMoveA", [open_mm, float(hand["open_speed_percent"]), "Next"])
    controller.wait_hand(lambda: abs(controller.hand("HandCurPos")-open_mm) < .4 and
                         not controller.hand("HandHoldState"))
    after = controller.stable(powered=True, empty=True)
    require(np.max(np.abs(np.asarray(after["joints"])-before["joints"])) < .05,
            "Arm moved during gripper opening; stopped")
    write_json(folder/"startup_gripper_open.json",
               {"stage": "empty_verified", "before": before, "after": after})
    return after


def run_calibration(rdk, cfg, vision, setup, folder):
    require(cfg["observation"].get("joints") and cfg["observation"].get("pose"),
            "No saved VIEW yet. Run the set-view mode first, then calibrate.")
    # Use the camera helper's normal timeout for sustained stationary capture.
    # A timed-out snapshot still stops the run before another movement.
    camera = Camera(vision, folder, snapshot_timeout=FIXED_CALIBRATION["snapshot_timeout_seconds"])
    planner = None
    records = []
    try:
        with Controller(cfg, setup) as controller:
            state = controller.prepare_for_motion(folder, empty=False)
            state = ensure_gripper_open(controller, setup, folder)
            report = controller.speed_report()
            write_json(folder/"speed_report.json", report)
            write_json(folder/"start_state.json", {"state": state, "speeds": report,
                       "setup": setup, "frames": controller.frames()})
            planner = Planner(rdk, controller, setup)
            override_note = controller.external_override_note()
            confirm("Start automatic calibration at command speed/acceleration/deceleration "
                    "100%. Current external overrides (unchanged): " + str(controller.initial_external) +
                    ((" " + override_note) if override_note else "") +
                    ". Keep the robot's swept area clear and the physical stop accessible. "
                    "The program will verify the saved VIEW and return to it if needed. "
                    "Confirm the original untrimmed Letter board is flat in its measured position "
                    "and the cube is removed. Keep the board and locked optics fixed. "
                    "The initial return "
                    "uses the temporary 35 mm region. If starting outside it, a separate "
                    "confirmation may reauthorize the last matching measured rectangle "
                    "for this return only. Small current-pose response probes "
                    "have a separate confirmation and stay inside that region. This approval "
                    "also authorizes checked fit/check collection and final return after the "
                    "response and fixed paper direction pass. Collection stops for a fresh "
                    "image every segment (up to 8 mm / 2 degrees), with a 2 mm response gate. "
                    "The arm/camera can extend beyond the paper. "
                    "All fixtures/cables must fit the model or be clear of the swept area.")
            controller.take()
            current = controller.stable(powered=True, empty=True)
            if np.max(np.abs(np.asarray(current["joints"])-cfg["observation"]["joints"])) < .05:
                view_check(current, cfg)
                planner.sync_check(current)
                require(planner.valid(current["joints"]), "Saved VIEW fails workspace/collision checks")
                log("Already at saved VIEW; skipping the redundant return move.")
            else:
                restored_anchor = confirm_previous_workspace_return(camera, controller, planner, cfg, setup, folder)
                route_to_view(planner, controller, cfg)
                if restored_anchor is not None:
                    planner.workspace = None
                    planner.calibration_anchor_xy = restored_anchor
                    planner.cache.clear()
            anchor, guide = acquire_calibration_board(camera, controller, planner, cfg, setup, folder)
            records = collect_guided_views(camera, controller, planner, anchor, guide, setup, folder)
            write_json(folder/"observations.json", records)
            # Separate final observation checks that the board/mount stayed stationary.
            current = controller.stable(powered=True)
            final_route = planner.route(current["joints"], anchor["state_after"]["joints"])
            write_json(folder/"final_anchor_route.json", final_route)
            planner.execute(final_route, "Return to acquired calibration view")
            final_check = camera.observation(controller, "final_anchor_check", "check")
            require(final_check["accepted"], "Fixed board is no longer visible at the acquired "
                    "calibration view: " + final_check["reason"])
            records.append(final_check)
            write_json(folder/"observations.json", records)
            result = fit_calibration(records, setup, folder)
            result["binding"] = {"observation": cfg["observation"], "frames": controller.frames(),
                                 "vision_fingerprint": fingerprint(vision),
                                 "geometry_fingerprint": fingerprint(geometry_binding(setup)),
                                 "geometry": geometry_binding(setup),
                                 "robot_ip": cfg["robot_ip"], "camera_ip": vision["camera_ip"]}
            result["binding"]["controller_reference"] = controller.reference_report
            write_json(folder/"calibration_candidate.json", result)
            log(json.dumps(result["quality"], indent=2))
            require(result["quality"]["passed"],
                    "Calibration did not meet table/quality/coverage limits. Candidate and all images saved; "
                    "previous accepted calibration remains unchanged. See QUALITY_AND_FORMAT.md.")
            result["workspace"] = register_printed_paper(camera, controller, result, folder)
            planner.workspace = result["workspace"]
            planner.cache.clear()
            route_to_view(planner, controller, cfg)
            check_stop()
            # Keep accepted files immutable. If the config write fails, its old
            # reference still identifies the untouched previous calibration.
            accepted_path = ROOT/"calibrations"/(stamp()+"_calibration_v2.json")
            write_json(accepted_path, result)
            CONFIG["calibration_v2"] = {"schema": SCHEMA,
                                        "path": accepted_path.relative_to(ROOT).as_posix(),
                                        "sha256": fingerprint(result)}
            write_json(CONFIG_FILE, CONFIG, backup=True)
            log("Calibration accepted and saved; at VIEW. No pick has run. "
                "Remove the board, put down one blue cube, then select Preview or Pick.")
    finally:
        try:
            camera.close()
        except Exception as exc:
            log("Camera cleanup: " + str(exc))
        finally:
            try:
                if planner:
                    planner.close()
            finally:
                # Diagnostics only; save_timings() never raises or masks the
                # original failure or cancellation.
                save_timings(folder)


def compact_cartesian_path(planner, path, h_start, h_end):
    """Retain dense IK checks, then merge only verified short straight segments."""
    line_start = h_start[:3, 3]
    line = h_end[:3, 3]-line_start
    line_squared = max(float(line @ line), 1e-10)
    positions = [planner.fk(q)[:3, 3] for q in path]
    compact = [path[0]]
    start = 0
    while start < len(path)-1:
        check_stop()
        end = start+1
        # Merge up to 10 mm / 10 deg of verified straight motion per commanded
        # step (was 5 mm / 5 deg). Every merged segment is still sampled by the
        # straightness test below and by planner.edge(), and the compact path is
        # preflighted before motion; this only reduces the number of stops.
        while end+1 < len(path) and np.linalg.norm(positions[end+1]-positions[start]) <= 10:
            end += 1
        while end > start+1:
            a, b = np.asarray(path[start]), np.asarray(path[end])
            samples = planner.samples(a, b)
            if np.max(np.abs(b-a)) < 10:
                poses = [planner.fk(q) for q in samples]
                nearest = [line_start+line*np.clip(np.dot(h[:3, 3]-line_start, line)/
                                                 line_squared, 0, 1) for h in poses]
                straight = all(np.linalg.norm(h[:3, 3]-p) < .2 and
                               rotation_error(h, h_start) <= planner.setup["planning"]["angle_tolerance_deg"]
                               for h, p in zip(poses, nearest))
                # This is a new direct path. Check its entire sampled collision
                # geometry before retaining it; the normal full preflight follows.
                if straight and planner.edge(a, b):
                    break
            end -= 1
        compact.append(path[end])
        start = end
    return compact


def cartesian_joint_path(planner, h_start, h_end, q_start):
    with TIMINGS.phase("cartesian_joint_path.total"):
        return _cartesian_joint_path(planner, h_start, h_end, q_start)


def _cartesian_joint_path(planner, h_start, h_end, q_start):
    require(rotation_error(h_start, h_end) < 1e-4, "Straight approach must keep grasp orientation")
    count = max(1, int(math.ceil(np.linalg.norm(h_end[:3, 3]-h_start[:3, 3])/1.0)))
    path = [list(map(float, q_start))]
    for i in range(1, count+1):
        target = h_start.copy()
        target[:3, 3] += (h_end[:3, 3]-h_start[:3, 3])*i/count
        q = planner.ik(target, np.asarray(path[-1]))
        require(np.max(np.abs(q-path[-1])) < 5, "IK branch jump in approach/lift")
        require(planner.edge(path[-1], q), "Approach/lift collision")
        # Joint subsegments approximate a Cartesian normal move; bound departure.
        a, b = planner.fk(path[-1])[:3, 3], planner.fk(q)[:3, 3]
        for sample in planner.samples(path[-1], q):
            p = planner.fk(sample)[:3, 3]
            v = b-a
            nearest = a+v*np.clip(np.dot(p-a, v)/max(np.dot(v, v), 1e-10), 0, 1)
            require(np.linalg.norm(p-nearest) < .2, "Joint approximation deviates from straight approach")
        path.append(q.tolist())
    # The dense path is already validated step by step: planner.ik() checks
    # controller range and J2P at every dense waypoint, planner.edge() checks
    # every dense segment, and the compact path is preflighted before motion.
    # Re-preflighting the dense path repeated those checks on joints that are
    # never commanded.
    compact = compact_cartesian_path(planner, path, h_start, h_end)
    log("Straight path: %d dense steps checked; %d commanded steps after bounded merging." %
        (len(path)-1, len(compact)-1))
    return compact


def plan_pick(planner, controller, cube, cal, cfg, setup):
    n = np.asarray(cal["table"]["normal_base"])
    y0 = np.asarray(cube["closing_axis_base"])
    fg = planner.grasp_tcp
    seed = np.asarray(controller.stable()["joints"])
    clearance, lift = float(cfg["approach_clearance_mm"]), float(cfg["lift_mm"])
    require(clearance > 0 and lift > 0, "Approach and lift distances must be positive")
    errors = []
    for angle in [0, math.pi/2, math.pi, -math.pi/2]:
        planner.set_payload(None)
        planner.set_target_cube(cube, cal)
        try:
            log("Planning grasp orientation %.0f deg: approach, descent, lift and held return." %
                math.degrees(angle))
            y = y0*math.cos(angle)+np.cross(n, y0)*math.sin(angle)
            z = -n
            x = np.cross(y, z)
            bg = transform(np.column_stack([x, y, z]), cube["centre_base_mm"])
            grasp = bg @ inv(fg)
            approach, lifted = grasp.copy(), grasp.copy()
            approach[:3, 3] += n*clearance
            lifted[:3, 3] += n*lift
            # Try the validated IK branches for a clear straight approach first;
            # the nearest solution and the full RRT remain the fallback.
            qa, to_approach = planner.route_to_ik(approach, seed)
            down = cartesian_joint_path(planner, approach, grasp, qa)
            for label, path in (("planned approach", to_approach), ("planned descent", down)):
                planner.preflight(path, label)
            planner.set_target_cube(None)
            # Open/closed swept finger envelope remains in the model throughout.
            up = cartesian_joint_path(planner, grasp, lifted, down[-1])
            w, h = setup["cube_width_mm"], setup["cube_height_mm"]
            cube_in_grasp = box_vertices([[-w/2, -w/2, -h/2], [w/2, w/2, h/2]])
            in_flange = points(fg, cube_in_grasp)
            payload = [in_flange.min(0).tolist(), in_flange.max(0).tolist()]
            # The cube initially contacts the table. Verify its geometric lowest point
            # never descends below the table; enable inflated payload after lift clearance.
            table_point = np.asarray(cal["table"]["point_base_mm"])
            for a, b in zip(up, up[1:]):
                for q in planner.samples(a, b):
                    verts = points(planner.fk(q), in_flange)
                    require(np.min((verts-table_point) @ n) >= -.2, "Lift carries cube below table")
            planner.preflight(up, "planned lift")
            planner.set_payload(payload)
            back = planner.route(up[-1], cfg["observation"]["joints"][:6])
            planner.preflight(back, "planned held return")
            # Check all other objects even while the cube is leaving table contact.
            payload_item = planner.held_box[0]
            planner.enable_pair(payload_item, planner.obstacles[0], on=False)
            planner.cache.clear()
            for a, b in zip(up, up[1:]):
                for q in planner.samples(a, b):
                    require(planner.valid(q), "Carried cube collides during lift")
            planner.enable_pair(payload_item, planner.obstacles[0])
            planner.cache.clear()
            planner.set_payload(None)
            planner.set_target_cube(cube, cal)
            return {"to_approach": to_approach, "down": down, "up": up, "back": back,
                    "payload_bounds_flange_mm": payload, "T_base_flange_grasp": grasp.tolist(),
                    "T_flange_grasp": fg.tolist(),
                    "object_size_mm": [setup["cube_width_mm"], setup["cube_width_mm"], setup["cube_height_mm"]],
                    "cube": cube, "start_joints": seed.tolist(),
                    "speed": setup["motion"], "collision_sample_deg": planner.step}
        except ValueError as exc:
            errors.append(str(exc))
    planner.set_payload(None)
    planner.set_target_cube(None)
    raise ValueError("No feasible grasp orientation/route, including held return to VIEW: " +
                     " | ".join(errors))


def run_pick(rdk, cfg, vision, setup, folder, execute):
    log("Operation selected: %s." % ("PICK - physical motion after approval and route checks" if execute
                                     else "PREVIEW - image capture and planning"))
    link = cfg.get("calibration_v2")
    require(link and link["schema"] == SCHEMA, "Run automatic calibration first")
    cal = read_json(ROOT/link["path"])
    require(fingerprint(cal) == link["sha256"], "Calibration file/config binding mismatch")
    camera = Camera(vision, folder)
    planner = None
    try:
        with Controller(cfg, setup) as controller:
            bind_calibration(cal, cfg, vision, setup, controller.frames())
            # Pick may start with a latched HandHoldState after manual removal.
            # Confirmed opening below must verify empty before any arm movement.
            state = (controller.prepare_for_motion(folder, empty=False) if execute
                     else ensure_gripper_open(controller, setup, folder))
            report = controller.speed_report()
            write_json(folder/"speed_report.json", report)
            require("workspace" in cal and "controller_reference" in cal["binding"],
                    "Recalibrate with the reference-corrected version and mark the paper boundary first")
            with TIMINGS.phase("pick.planner_setup"):
                planner = Planner(rdk, controller, setup, cal["table"], cal["workspace"],
                                  grasp_transform(setup))
            old_ref = rigid(cal["binding"]["controller_reference"]["T_flange_controller_reference"])
            require(np.linalg.norm(old_ref[:3, 3]-controller.T_flange_controller[:3, 3]) < .5 and
                    rotation_error(old_ref, controller.T_flange_controller) < .3,
                    "Controller reference differs from calibration")
            if execute:
                hand = setup["gripper"]
                override_note = controller.external_override_note()
                confirm("PICK AND LIFT at command speed/acceleration/deceleration 100%. "
                        "External overrides remain " + str(controller.initial_external) +
                        ((" " + override_note) if override_note else "") +
                        f". OK FIRST OPENS THE GRIPPER at its current pose to {hand['open_position_mm']:.1f} mm. "
                        "Anything still held will be released; remove the previous cube or "
                        "provide support below it before approval. Keep hands clear. "
                        "The arm returns to VIEW only after the gripper verifies empty" +
                        ". Remove calibration board; place one cube on the same table. "
                        "Camera mount/focus and table must be unchanged. The cube will remain "
                        "held when the robot returns to VIEW; there is no place/release step. "
                        "Confirm modelled swept area is clear and physical stop is accessible.")
                controller.take()
                before_open = controller.stable(powered=True)
                planner.sync_check(before_open)
                require(planner.valid(before_open["joints"]),
                        "Current pose fails gripper-sweep/collision checks; stopped before startup opening")
                require(setup["cube_width_mm"]+3 <= hand["open_position_mm"] <= 30,
                        "Startup opening must leave at least 3 mm above cube width and be <=30 mm")
                controller.check_frames()
                controller.check_motion_state()
                write_json(folder/"startup_gripper_open.json", {"stage": "opening", "before": before_open})
                log("Pick startup: opening gripper to %.1f mm at the current pose." % hand["open_position_mm"])
                controller.hand("HandMoveA", [float(hand["open_position_mm"]), float(hand["open_speed_percent"]), "Next"])
                controller.wait_hand(lambda: abs(controller.hand("HandCurPos")-hand["open_position_mm"]) < .4 and
                                     not controller.hand("HandHoldState"))
                after_open = controller.stable(powered=True, empty=True)
                require(np.max(np.abs(np.asarray(after_open["joints"])-before_open["joints"])) < .05,
                        "Arm moved during startup gripper opening; stopped before return to VIEW")
                write_json(folder/"startup_gripper_open.json",
                           {"stage": "empty_verified", "before": before_open, "after": after_open})
                log("Pick startup: gripper open and empty verified. Preparing return to VIEW.")
                route_to_view(planner, controller, cfg)
            else:
                view_check(state, cfg)
            fits = []
            previous_state = None
            for index in range(2):
                # The previous frame's verified post-exposure state is this
                # frame's stationary boundary reading; frame() still takes its
                # own post-exposure stable() and still stops if the joints moved.
                frame, rec, prefix = camera.frame("pick_%d" % index, controller,
                                                  before=previous_state)
                previous_state = rec["state_after"]
                view_check(rec["state_after"], cfg)
                with TIMINGS.phase("pick.locate_cube"):
                    fit, drawn = locate_cube(frame, vision, cal, rec["state_after"], setup, prefix)
                write_json(prefix.with_name(prefix.name+"_fit.json"), fit)
                cv2.imwrite(str(prefix.with_name(prefix.name+"_fit.png")), drawn)
                fits.append(fit)
                pause(.3)
            require(np.linalg.norm(np.asarray(fits[0]["centre_base_mm"])-fits[1]["centre_base_mm"]) < .8,
                    "Cube moved or localization unstable between fresh captures")
            yaw_change = (fits[0]["yaw_table_rad"]-fits[1]["yaw_table_rad"]+math.pi/4) % (math.pi/2)-math.pi/4
            require(abs(yaw_change) < math.radians(3), "Cube yaw is unstable")
            log("Cube stable in two images. Preparing the full pick route; sampled checks can take a minute.")
            plan = plan_pick(planner, controller, fits[-1], cal, cfg, setup)
            write_json(folder/"pick_plan.json", plan)
            if not execute:
                log("Preview saved. No robot or gripper movement. Preview plans are never replayed; Pick captures again.")
                return
            log("Pick route prepared. Capturing final cube verification before opening the gripper.")
            # Planning may take time. Refresh perception before opening or moving away.
            frame, rec, prefix = camera.frame("pick_final_fresh_check", controller,
                                              before=previous_state)
            with TIMINGS.phase("pick.locate_cube"):
                latest, _ = locate_cube(frame, vision, cal, rec["state_after"], setup, prefix)
            require(np.linalg.norm(np.asarray(latest["centre_base_mm"])-fits[-1]["centre_base_mm"]) < .8,
                    "Cube changed during planning; discard plan")
            require(abs((latest["yaw_table_rad"]-fits[-1]["yaw_table_rad"]+math.pi/4) %
                        (math.pi/2)-math.pi/4) < math.radians(3), "Cube rotated during planning")
            view_check(controller.stable(powered=True), cfg)
            controller.check_frames()
            controller.check_motion_state()
            hand = setup["gripper"]
            require(setup["cube_width_mm"]+3 <= hand["open_position_mm"] <= 30,
                    "Stock opening must leave at least 3 mm above cube width and be <=30 mm")
            log("Final cube check passed. Opening gripper to %.1f mm." % hand["open_position_mm"])
            controller.hand("HandMoveA", [float(hand["open_position_mm"]), float(hand["open_speed_percent"]), "Next"])
            controller.wait_hand(lambda: abs(controller.hand("HandCurPos")-hand["open_position_mm"]) < .4)
            require(not controller.hand("HandHoldState"), "Gripper unexpectedly reports holding after opening")
            planner.execute(plan["to_approach"], "Approach cube", report_progress=True)
            planner.execute(plan["down"], "Descend along table normal", report_progress=True)
            controller.check_frames()
            controller.check_motion_state()
            controller.hand("HandMoveH", [float(hand["force_n"]), True, "Next"])
            controller.wait_hand(controller.grip_ok)
            pause(.3)
            require(controller.grip_ok(), "Grip did not remain confirmed")
            controller.holding_required = True
            planner.set_target_cube(None)
            planner.set_payload(plan["payload_bounds_flange_mm"])
            # Initial table contact is expected. The preflight above separately proved
            # nonpenetration along the lift; all remaining collision pairs stay active.
            planner.enable_pair(planner.held_box[0], planner.obstacles[0], on=False)
            planner.cache.clear()
            planner.execute(plan["up"], "Lift cube", report_progress=True)
            planner.enable_pair(planner.held_box[0], planner.obstacles[0])
            planner.cache.clear()
            planner.execute(plan["back"], "Return holding cube to VIEW", report_progress=True)
            view_check(controller.stable(powered=True), cfg)
            require(controller.grip_ok(), "Grip confirmation lost at VIEW")
            write_json(folder/"pick_result.json", {"status": "held_at_view", "completed_utc": stamp(),
                                                   "state": controller.state(), "speed_report": report})
            log("Pick/lift complete; back at VIEW, cube held. No release or motor-off command sent.")
    finally:
        try:
            camera.close()
        except Exception as exc:
            log("Camera cleanup: " + str(exc))
        finally:
            try:
                if planner:
                    planner.close()
            finally:
                if planner and planner.planning_notes:
                    try:
                        write_json(folder/"planning_diagnostics.json",
                                   {"program_revision": PROGRAM_REVISION, "session": folder.name,
                                    "notes": planner.planning_notes})
                    except Exception as exc:
                        log("Planning diagnostics could not be written (ignored): " + str(exc))
                # Diagnostics only; save_timings() never raises or masks the
                # original failure or cancellation.
                save_timings(folder)


def save_current_view(cfg, vision, setup):
    """Read real controller state and preview camera.

    The arm never moves. The gripper is opened only if it is not fully open,
    after the operator is told to remove any object and approves it.
    """
    folder = ROOT/"sessions"/(stamp()+"_set_view")
    camera = Camera(vision, folder)
    try:
        with Controller(cfg, setup) as controller:
            before = ensure_gripper_open(controller, setup, folder)
            # Controller reference has not yet been reconciled with the physical
            # model. Preserve raw Tool0/Work0 pose, exactly as for the old VIEW.
            camera.frame("current_view_preview")
            after = controller.stable(empty=True)
            require(np.max(np.abs(np.asarray(before["joints"])-after["joints"])) < .05,
                    "Robot moved during VIEW preview; nothing saved")
            log("Current controller pose: " + str(after["pose"]))
            log("Current controller joints: " + str(after["joints"]))
            confirm("SAVE CURRENT POSE AS VIEW - no robot movement. Confirm the camera "
                    "image shows your paper and the board is visible. Your measured paper "
                    "edges must be relative to THIS pose: 70 mm left, 140 mm right, "
                    "40 mm behind and 240 mm toward you. The old VIEW is backed up and "
                    "any previous calibration binding is invalidated. OK saves; Cancel "
                    "leaves configuration unchanged.")
            latest = controller.stable(empty=True)
            require(np.max(np.abs(np.asarray(after["joints"])-latest["joints"])) < .05,
                    "Robot moved while confirming VIEW; nothing saved")
            new_observation = {"pose": latest["pose"], "joints": latest["joints"],
                               "tool": 0, "work": 0}
            write_json(folder/"new_view.json", {"old_observation": cfg["observation"],
                       "new_observation": new_observation, "frames": controller.frames()})
            CONFIG["observation"] = new_observation
            CONFIG.pop("calibration_v2", None)
            write_json(CONFIG_FILE, CONFIG, backup=True)
            log("New VIEW saved. No motion performed. Run calibrate separately to collect new data.")
    finally:
        camera.close()
        cv2.destroyAllWindows()


@contextmanager
def calibration_keep_awake():
    """Temporary Windows idle-sleep request; does not change the power plan."""
    if os.name != "nt":
        yield
        return
    import ctypes
    set_state = ctypes.WinDLL("kernel32", use_last_error=True).SetThreadExecutionState
    set_state.argtypes = [ctypes.c_uint32]
    set_state.restype = ctypes.c_uint32
    # ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED.
    previous = set_state(0x80000003)
    require(previous != 0, "Could not request Windows keep-awake; stopped before robot connection")
    try:
        log("Windows keep-awake request active during calibration. Keep the laptop lid open; "
            "manual Sleep/power actions can still interrupt the run.")
        yield
    finally:
        try:
            # Restore this thread's previous request, including on cancel/error.
            if not set_state(previous | 0x80000000):
                log("Windows keep-awake cleanup was not confirmed")
        except Exception as exc:
            log("Windows keep-awake cleanup: " + str(exc))


def choose_operation(default_mode):
    """Modal dropdown of the operations.

    Returns the selected mode string, False when cancelled, or None when Tk is
    unavailable so the caller can fall back to the previous typed dialog.
    """
    try:
        import tkinter as tk
        from tkinter import ttk
    except Exception as exc:
        log("Operation dropdown unavailable (%s); using the text dialog." % exc)
        return None
    root, choice = None, {"mode": False}
    try:
        root = tk.Tk()
        root.title("COBOTTA operation")
        root.resizable(False, False)
        try:
            root.attributes("-topmost", True)
        except Exception:
            pass
        tk.Label(root, justify="left", padx=14, pady=10, text=(
            "Choose the operation, then press OK.\n"
            "preview: check the cube and plan, no motion.\n"
            "pick: physical pick/lift, with its own confirmations.\n"
            "calibrate: collect a fresh calibration.\n"
            "status / set-view / board / instructions: no arm motion.")).pack(anchor="w")
        box = ttk.Combobox(root, values=list(OPERATIONS), state="readonly", width=28)
        box.set(default_mode if default_mode in OPERATIONS else OPERATIONS[0])
        box.pack(padx=14, pady=(0, 10), fill="x")

        def accept(*_):
            choice["mode"] = str(box.get()).strip().lower()
            root.destroy()

        def cancel(*_):
            root.destroy()

        buttons = tk.Frame(root)
        buttons.pack(padx=14, pady=(0, 12), fill="x")
        tk.Button(buttons, text="Cancel", width=10, command=cancel).pack(side="right")
        tk.Button(buttons, text="OK", width=10, command=accept).pack(side="right", padx=(0, 6))
        root.bind("<Return>", accept)
        root.bind("<Escape>", cancel)
        root.protocol("WM_DELETE_WINDOW", cancel)
        root.update_idletasks()
        x = max(0, (root.winfo_screenwidth()-root.winfo_width())//2)
        y = max(0, (root.winfo_screenheight()-root.winfo_height())//2)
        root.geometry("+%d+%d" % (x, y))
        box.focus_set()
        root.mainloop()
        return choice["mode"]
    except Exception as exc:
        log("Operation dropdown failed (%s); using the text dialog." % exc)
        return None
    finally:
        if root is not None:
            try:
                root.destroy()
            except Exception:
                pass


def main(forced_mode=None):
    global RDK, STOP, STOP_REASON, _WINDOW_READY
    STOP = False
    STOP_REASON = None
    print("COBOTTA project folder:", ROOT, flush=True)
    print("COBOTTA Python revision:", PROGRAM_REVISION, flush=True)
    signal.signal(signal.SIGINT, stop_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, stop_signal)
    require(not (ROOT/"STOP_COBOTTA").exists(),
            "STOP_COBOTTA latch exists. Resolve stop reason and remove it manually before restarting.")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=list(OPERATIONS))
    args = parser.parse_args() if forced_mode is None else argparse.Namespace(mode=forced_mode)
    RDK = robolink.Robolink()
    mode = args.mode
    operations = OPERATIONS
    if mode is None:
        # Choose the operation once from a dropdown. Routine fixed-setup
        # calibration prompts remain automated; explicit launch arguments still
        # bypass this menu. If Tk is unavailable, the old typed dialog is used.
        selected = FIXED_CALIBRATION["default_mode"]
        while mode not in operations:
            picked = choose_operation(selected)
            if picked is None:
                picked = robodialogs.InputDialog(
                    "Type exactly one mode: preview, pick, calibrate, status, set-view, board, instructions. "
                    "Preview checks the cube and plan; pick requests a physical pick/lift.",
                    selected, title="COBOTTA operation")
                if picked is None or picked is False:
                    raise Stopped("Cancelled. No operation started.")
            elif picked is False:
                raise Stopped("Cancelled. No operation started.")
            mode = str(picked).strip().lower()
            selected = mode
            if mode not in operations:
                log("Unknown operation %r. Enter one of: %s" % (mode, ", ".join(operations)))
    require(mode in operations, "Unknown operation")
    if mode in ("board", "instructions"):
        path = ROOT/(PRINTABLE_BOARD if mode == "board" else "docs/OPERATING.md")
        if mode == "board":
            require(board_pdf_matches(path),
                    "The board PDF has changed; restore the supplied US Letter sheet")
        log(str(path))
        os.startfile(str(path))
        return
    cfg = {"robot_ip": CONFIG["robot_ip"], "observation": CONFIG["observation"],
           "speed_percent": CONFIG["speed_percent"],
           "approach_clearance_mm": CONFIG["approach_clearance_mm"],
           "lift_mm": CONFIG["lift_mm"]}
    if CONFIG.get("calibration_v2"):
        cfg["calibration_v2"] = CONFIG["calibration_v2"]
    vision = {"camera_ip": CONFIG["camera_ip"],
              "camera_username": CONFIG["camera_username"],
              "camera_password": CONFIG["camera_password"]}
    vision.update(CONFIG.get("vision", {}))
    setup = read_json(ROOT/"automatic_setup.json")
    require(cfg["observation"]["tool"] == 0 and cfg["observation"]["work"] == 0,
            "This workflow uses saved flange Tool 0 / base Work 0")
    command_settings = array([setup["motion"].get(k) for k in
                              ("speed_percent", "acceleration_percent", "deceleration_percent")],
                             (3,), "arm command speed/acceleration/deceleration")
    require(float(cfg["speed_percent"]) == 100 and np.all(command_settings == 100),
            "This delivery preserves the requested 100% settings. A changed speed requires explicit configuration review.")
    if mode == "set-view":
        save_current_view(cfg, vision, setup)
        return
    if mode == "status":
        with Controller(cfg, setup) as controller:
            state = controller.state()
            data = {"state": state, "frames": controller.frames(), "speeds": controller.speed_report(),
                    "start_OutRange": controller.range_code(state["joints"]),
                    "automatic_inward_recovery": "not verified for this controller/firmware; no bypass implemented"}
            log(json.dumps(data, indent=2))
            write_json(ROOT/"sessions"/(stamp()+"_status.json"), data)
        return
    folder = ROOT/"sessions"/(stamp()+"_"+mode)
    folder.mkdir(parents=True)
    try:
        validate_setup(setup, for_calibration=mode == "calibrate")
        if mode == "calibrate":
            with calibration_keep_awake():
                run_calibration(RDK, cfg, vision, setup, folder)
        else:
            run_pick(RDK, cfg, vision, setup, folder, execute=mode == "pick")
    except BaseException as exc:
        failure = {"time_utc": stamp(), "type": type(exc).__name__,
                   "program_revision": PROGRAM_REVISION,
                   "message": str(exc), "traceback": traceback.format_exc()}
        # Preserve the fresh pendant-override readings that explain a startup stop.
        try:
            if (folder/"speed_report.json").is_file():
                failure["speed_report"] = read_json(folder/"speed_report.json")
        except Exception as report_error:
            log("speed_report.json could not be added to failure.json: " + str(report_error))
        write_json(folder/"failure.json", failure)
        log("STOPPED: " + str(exc))
        raise
    finally:
        # Snapshot windows are image viewers only. No custom application UI is created.
        cv2.destroyAllWindows()
        _WINDOW_READY = False


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        print("COBOTTA Python ended:", exc, file=sys.stderr, flush=True)
        if not isinstance(exc, SystemExit):
            robodialogs.ShowMessage(str(exc), title="COBOTTA Python - stopped")
        raise
