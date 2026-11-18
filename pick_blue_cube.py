"""Pick entry point delegates to cobotta_calibrate_pick's v1 calibration.

Historical helper definitions below are retained for legacy module imports only.
Run the unified RoboDK entry point for all new calibration and picking.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
# This compatibility entry can also be copied to a temporary file by RoboDK.
# Find the full project before importing its helpers. No robot is connected here.
PROJECT_DIRECTORY = r"C:\path\to\cobottaPickAndLift"
_override = os.environ.get("COBOTTA_PROJECT_DIR", "").strip()
_candidates = ([Path(_override).expanduser()] if _override else
               [Path(__file__).resolve().parent, Path(PROJECT_DIRECTORY), Path.cwd(),
                Path.cwd()/"cobotta_blue_pick", Path.cwd()/"outputs"/"cobotta_blue_pick"])
_required = ("bcapclient.py", "variant.py", "orinexception.py", "http_camera.py",
             "detect_blue.py", "cobotta_calibrate_pick.py", "pick_config.json",
             "vision_config.json", "automatic_setup.json", "charuco-7x5.pdf",
             "charuco-7x5-letter.pdf")
ROOT = next((p.resolve() for p in _candidates
             if all((p/name).is_file() for name in _required)), None)
if ROOT is None:
    raise RuntimeError("COBOTTA project files not found. Set PROJECT_DIRECTORY or "
                       "COBOTTA_PROJECT_DIR to the complete cobotta_blue_pick folder.")
sys.path.insert(0, str(ROOT))
from bcapclient import BCAPClient
from http_camera import HTTPCamera
from detect_blue import detect, overlay



def require(condition, message):
    if not condition:
        raise ValueError(message)


def numbers(value, shape, label):
    require(value is not None, 'Missing ' + label)
    a = np.asarray(value, dtype=float)
    require(a.shape == shape and np.isfinite(a).all(), 'Invalid ' + label)
    return a


def close_pose(actual, expected, xyz=0.5, deg=0.5):
    a = numbers(actual, (7,), 'actual pose')
    b = numbers(expected, (7,), 'expected pose')
    angles = (a[3:6] - b[3:6] + 180) % 360 - 180
    return (np.linalg.norm(a[:3]-b[:3]) <= xyz
            and np.max(np.abs(angles)) <= deg and a[6] == b[6])


def project(h, pixels):
    p = np.asarray(pixels, dtype=float).reshape(-1, 2)
    q = np.column_stack([p, np.ones(len(p))]) @ h.T
    require(np.isfinite(q).all() and np.all(np.abs(q[:, 2]) > 1e-8),
            'Invalid calibration projection')
    return q[:, :2] / q[:, 2:3]


def calibration(cfg):
    cal = cfg['calibration']
    fit = np.asarray(cal['fit_pixel_xy_to_robot_xy'], dtype=float)
    check = np.asarray(cal['check_pixel_xy_to_robot_xy'], dtype=float)
    require(fit.ndim == 2 and fit.shape[1:] == (4,) and len(fit) >= 6,
            'Calibration requires at least six fit rows: [pixel_x,pixel_y,robot_x,robot_y]')
    require(check.ndim == 2 and check.shape[1:] == (4,) and len(check) >= 3,
            'Calibration requires at least three separate check rows')
    require(np.isfinite(fit).all() and np.isfinite(check).all(), 'Non-finite calibration data')
    require(len(np.unique(fit[:, :2], axis=0)) == len(fit), 'Repeated fit locations')
    require(len(np.unique(check[:, :2], axis=0)) == len(check), 'Repeated check locations')
    for p in check[:, :2]:
        require(np.min(np.linalg.norm(fit[:, :2]-p, axis=1)) > 5,
                'Check locations must be independent of fit locations')
    hull = cv2.convexHull(fit[:, :2].astype(np.float32))
    require(cv2.contourArea(hull) >= 2500, 'Calibration coverage too small or collinear')
    h, _ = cv2.findHomography(fit[:, :2], fit[:, 2:], method=0)
    require(h is not None and np.isfinite(h).all() and abs(np.linalg.det(h)) > 1e-12,
            'Degenerate calibration')
    # A projective horizon cannot cross the taught area.
    denominators = np.column_stack([fit[:, :2], np.ones(len(fit))]) @ h[2]
    require(np.all(denominators > 1e-8) or np.all(denominators < -1e-8),
            'Calibration horizon crosses the working area')
    for p in check[:, :2]:
        require(cv2.pointPolygonTest(hull, tuple(map(float, p)), False) >= 0,
                'Check locations must be inside the fit coverage')
    errors = {}
    for name, data in [('fit', fit), ('check', check)]:
        error = np.linalg.norm(project(h, data[:, :2])-data[:, 2:], axis=1)
        errors[name + '_max_error_mm'] = float(np.max(error))
        require(np.max(error) <= 1.0, name + ' calibration error exceeds 1 mm')
    return h, hull, errors


class Robot:
    """Thin adapter; constructor and state() do not enable or move hardware."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.client = None
        self.ctrl = self.arm = None
        self.started = self.owned = False

    def __enter__(self):
        try:
            self.client = BCAPClient(self.cfg['robot_ip'], 5007, 10.0)
            self.client.service_start('')
            self.started = True
            self.ctrl = self.client.controller_connect('', 'CaoProv.DENSO.VRC', 'localhost', '')
            self.arm = self.client.controller_getrobot(self.ctrl, 'Arm', '')
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def variable(self, name):
        h = self.client.robot_getvariable(self.arm, name, '')
        try:
            return self.client.variable_getvalue(h)
        finally:
            self.client.variable_release(h)

    def hand(self, command, arguments=None):
        return self.client.controller_execute(self.ctrl, command, arguments)

    def state(self):
        return {'pose': self.variable('@CURRENT_POSITION'),
                'joints': self.variable('@CURRENT_ANGLE'),
                'tool': self.variable('@CURRENT_TOOL'),
                'work': self.variable('@CURRENT_WORK'),
                'servo_on': self.variable('@SERVO_ON'),
                'busy': self.variable('@BUSY_STATUS'),
                'hand_position': self.hand('HandCurPos'),
                'hand_holding': self.hand('HandHoldState'),
                'hand_busy': self.hand('HandBusyState')}

    def take_arm(self):
        # Internal and external percentages multiply; use the requested value for both.
        speed = float(self.cfg['speed_percent'])
        require(math.isfinite(speed) and 0 < speed <= 100, 'Speed must be between 0 and 100 percent')
        self.client.robot_execute(self.arm, 'TakeArm', [0, 0])
        self.owned = True
        self.client.robot_execute(self.arm, 'ExtSpeed', [speed, speed, speed])

    def move(self, pose):
        # DENSO interpolation 2 = linear. @E waits for the end-position condition.
        speed = float(self.cfg['speed_percent'])
        self.client.robot_move(self.arm, 2, [list(map(float, pose)), 'P', '@E'],
                               'SPEED={0:g},ACCEL={0:g},DECEL={0:g}'.format(speed))
        require(close_pose(self.variable('@CURRENT_POSITION'), pose),
                'Robot did not reach the commanded pose')

    def halt(self):
        self.client.robot_halt(self.arm, '')

    def __exit__(self, *_):
        if self.client is None:
            return
        operations = []
        if self.owned:
            operations.append(lambda: self.client.robot_execute(self.arm, 'GiveArm', None))
        if self.arm is not None:
            operations.append(lambda: self.client.robot_release(self.arm))
        if self.ctrl is not None:
            operations.append(lambda: self.client.controller_disconnect(self.ctrl))
        if self.started:
            operations.append(self.client.service_stop)
        for action in operations:
            try:
                action()
            except Exception as exc:
                print('Cleanup warning:', exc, file=sys.stderr)
        # Intentionally no automatic gripper release or motor-off operation.


def check_observation(state, cfg, powered=False):
    obs = cfg['observation']
    require(obs is not None, 'Record the observation pose first')
    require(not state['busy'] and not state['hand_busy'], 'Robot or gripper is moving')
    require(state['tool'] == obs['tool'] and state['work'] == obs['work'], 'Tool/work frame changed')
    require(close_pose(state['pose'], obs['pose']), 'Manually return to the taught observation pose')
    actual_joints = np.asarray(state['joints'], dtype=float)
    saved_joints = np.asarray(obs['joints'], dtype=float)
    require(actual_joints.shape == saved_joints.shape and np.isfinite(actual_joints).all()
            and np.max(np.abs(actual_joints-saved_joints)) <= 0.5,
            'Observation joint configuration changed')
    require(not state['hand_holding'], 'Gripper already reports a held object')
    if powered:
        require(bool(state['servo_on']), 'Prepare/enable the robot using its normal controls first')


def check_calibration_binding(robot, cfg, vision):
    """Honor the settings recorded by the guided calibration exporter."""
    binding = cfg['calibration'].get('provenance')
    if not binding:
        return
    signature = hashlib.sha256(json.dumps(vision, sort_keys=True, allow_nan=False).encode()).hexdigest()
    require(signature == binding['vision_fingerprint'],
            'Detection settings changed after calibration. Recalibrate before using this mapping.')
    for label, prefix, number in [('tool', 'TOOL', cfg['observation']['tool']),
                                  ('work', 'WORK', cfg['observation']['work'])]:
        actual = np.asarray(robot.variable(prefix + str(number)), dtype=float)
        expected = np.asarray(binding['frame_definitions'][label], dtype=float)
        require(actual.shape == expected.shape and np.isfinite(actual).all()
                and np.allclose(actual, expected, atol=1e-6, rtol=0),
                'The ' + label + ' definition changed after calibration. Recalibrate.')


def capture(vision):
    camera = HTTPCamera(vision['camera_ip'], vision['camera_username'],
                        os.environ.get('COBOTTA_CAMERA_PASSWORD', 'password'))
    try:
        frame = cv2.imdecode(np.frombuffer(camera.capture(), np.uint8), cv2.IMREAD_COLOR)
        require(frame is not None, 'Camera image decoding failed')
        return frame
    finally:
        camera.close()


def make_plan(result, cfg):
    require(result['status'] == 'single_candidate', 'Need exactly one blue candidate: ' + result['status'])
    require(result['image_size_wh'] == cfg['calibration']['image_size_wh'], 'Image resolution changed')
    h, hull, errors = calibration(cfg)
    centre = result['candidates'][0]['centre_px']
    require(cv2.pointPolygonTest(hull, tuple(map(float, centre)), True) >= 10,
            'Cube centre is outside the calibrated area or too close to its edge')
    obs = numbers(cfg['observation']['pose'], (7,), 'observation pose')
    grasp = numbers(cfg['reference_grasp_pose'], (7,), 'reference grasp pose').copy()
    # Fixed orientation: cube must be aligned like the calibration cube.
    angles = (obs[3:6]-grasp[3:6]+180) % 360 - 180
    require(np.max(np.abs(angles)) <= 0.5 and obs[6] == grasp[6],
            'This first version requires matching observation/grasp orientation and configuration')
    reference_angle = cfg['reference_cube_image_angle_deg']
    require(reference_angle is not None and math.isfinite(reference_angle), 'Missing reference cube image angle')
    box = np.asarray(result['candidates'][0]['box_px'])
    edge = box[1]-box[0]
    angle = math.degrees(math.atan2(edge[1], edge[0]))
    angle_error = abs((angle-reference_angle+45) % 90-45)
    require(angle_error <= 5.0, 'Cube is rotated relative to the taught grasp; align it first')
    grasp[:2] = project(h, [centre])[0]
    clearance = float(cfg['approach_clearance_mm'])
    lift = float(cfg['lift_mm'])
    require(20 <= clearance <= 80 and 5 <= lift <= clearance, 'Invalid approach/lift distances')
    approach = grasp.copy()
    approach[2] += clearance
    above = grasp.copy()
    above[2] = obs[2]
    lifted = grasp.copy()
    lifted[2] += lift
    require(obs[2] >= approach[2], 'Observation pose is below the approach height')
    limits = numbers(cfg['clear_tcp_box_xyz_mm'], (3, 2), 'verified clear TCP box')
    require(np.all(limits[:, 1] > limits[:, 0]), 'Invalid TCP bounds')
    poses = {'above': above, 'approach': approach, 'grasp': grasp, 'lift': lifted}
    for name, pose in [('observation', obs)] + list(poses.items()):
        require(np.all(pose[:3] >= limits[:, 0]) and np.all(pose[:3] <= limits[:, 1]),
                name + ' pose is outside the verified clear TCP box')
    return {'poses': {k: v.tolist() for k, v in poses.items()},
            'centre_px': centre, 'calibration_errors': errors,
            'cube_angle_error_deg': angle_error}


def commissioned(cfg):
    for field in ['geometry_and_route_verified', 'calibration_physically_verified', 'gripper_settings_verified']:
        require(cfg.get(field) is True, 'Commissioning incomplete: ' + field)
    require(math.isfinite(cfg['speed_percent']) and 0 < cfg['speed_percent'] <= 100,
            'Speed must be between 0 and 100 percent')
    hand = cfg['gripper']
    for field in ['open_position_mm', 'force_n']:
        require(hand[field] is not None and math.isfinite(hand[field]), 'Missing gripper ' + field)
    require(0 < hand['force_n'] <= 20, 'Invalid grip force')
    limits = numbers(hand['gripped_position_range_mm'], (2,), 'gripped position window')
    require(limits[1] > limits[0], 'Invalid gripped position window')
    require(not limits[0] <= hand['open_position_mm'] <= limits[1],
            'Open position overlaps the measured gripped position window')


def wait_hand(robot, predicate, timeout=6):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError('Gripper did not reach the required state within six seconds')


def grip_ok(robot, cfg):
    low, high = cfg['gripper']['gripped_position_range_mm']
    return bool(robot.hand('HandHoldState')) and low <= robot.hand('HandCurPos') <= high


def execute_pick(robot, cfg, vision):
    commissioned(cfg)
    check_calibration_binding(robot, cfg, vision)
    check_observation(robot.state(), cfg, powered=True)
    robot.take_arm()
    gripping = False
    try:
        check_observation(robot.state(), cfg, powered=True)
        time.sleep(1.0)
        # Capture twice, with pose checks, to reject unstable detections.
        frames = []
        plans = []
        for _ in range(2):
            check_observation(robot.state(), cfg, powered=True)
            frame = capture(vision)
            check_observation(robot.state(), cfg, powered=True)
            frames.append(frame)
            plans.append(make_plan(detect(frame, vision), cfg))
        require(np.linalg.norm(np.asarray(plans[0]['poses']['grasp'][:2])
                               - plans[1]['poses']['grasp'][:2]) <= 1.0,
                'Cube detection is unstable by more than 1 mm')
        plan = plans[-1]
        save_preview(frames[-1], detect(frames[-1], vision), plan)
        check_calibration_binding(robot, cfg, vision)
        check_observation(robot.state(), cfg, powered=True)
        hand = cfg['gripper']
        robot.hand('HandMoveA', [float(hand['open_position_mm']), 10.0, 'Next'])
        wait_hand(robot, lambda: not robot.hand('HandBusyState')
                  and abs(robot.hand('HandCurPos')-hand['open_position_mm']) <= 0.5)
        require(not robot.hand('HandHoldState'), 'Gripper still reports holding after opening')
        # No target updates after motion starts: the table and cube must remain stationary.
        for stage in ['above', 'approach', 'grasp']:
            robot.move(plan['poses'][stage])
        robot.hand('HandMoveH', [float(hand['force_n']), True, 'Next'])
        wait_hand(robot, lambda: grip_ok(robot, cfg))
        gripping = True
        time.sleep(0.3)
        require(grip_ok(robot, cfg), 'Grip did not remain confirmed')
        robot.move(plan['poses']['lift'])
        require(grip_ok(robot, cfg), 'Grip confirmation was lost during lift')
        print('Lift finished. No release or motor-off command sent. Support the cube before releasing it.')
    except BaseException:
        # Best effort only. A lost connection cannot guarantee a stop.
        try:
            robot.halt()
        except Exception as exc:
            print('Could not confirm robot halt; use physical stop:', exc, file=sys.stderr)
        if not gripping:
            try:
                robot.hand('HandStop')
            except Exception:
                pass
        raise


def save_preview(frame, result, plan=None):
    folder = ROOT / 'captures'
    folder.mkdir(exist_ok=True)
    cv2.imwrite(str(folder / 'pick_preview.png'), overlay(frame, result))
    (folder / 'pick_preview.json').write_text(json.dumps({'detection': result, 'plan': plan}, indent=2), encoding='utf-8')



def main():
    parser = argparse.ArgumentParser(description="COBOTTA v1 pick/calibration integration")
    parser.add_argument("mode", nargs="?", default="preview",
                        choices=["preview", "pick", "status", "calibrate"])
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.mode == "pick" and not args.execute:
        parser.error("Physical pick requires pick --execute; preview does not move the robot")
    from cobotta_calibrate_pick import main as unified_main
    unified_main(forced_mode=args.mode)


if __name__ == "__main__":
    main()
