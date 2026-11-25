"""Blue-object detection setup for COBOTTA. Contains no robot commands.

Run with RoboDK's Python. Default: one fresh camera capture and an overlay.
--live repeats captures; --image PATH operates entirely offline.
Pixel centres are observations, NOT robot coordinates or confirmed grasps.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

import cv2
import numpy as np
from http_camera import HTTPCamera

DEFAULT_GLARE_PROFILE = {"min_blue_fraction": 0.30, "max_saturation": 40,
                         "max_value": 100, "background_value_gap": 30}


def _hsv_bound(value, label):
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,) or not np.isfinite(arr).all():
        raise ValueError(label + " must be a 3-element numeric HSV array")
    if np.any(arr < 0) or np.any(arr > 255):
        raise ValueError(label + " values must lie in [0, 255]")
    return arr.astype(np.uint8)


def _read_image(path):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError("Image file not found: " + str(path))
    data = np.frombuffer(path.read_bytes(), np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def _write_image(path, image):
    path = Path(path)
    ok, encoded = cv2.imencode(path.suffix or ".png", image)
    if not ok:
        raise OSError("Could not encode " + path.name)
    path.write_bytes(encoded.tobytes())


def blue_mask(frame, config, glare_profile=None):
    """Blue segmentation with optional bounded recovery on the known workpiece."""
    lower = _hsv_bound(config["hsv_lower"], "hsv_lower")
    upper = _hsv_bound(config["hsv_upper"], "hsv_upper")
    if np.any(lower > upper):
        raise ValueError("hsv_lower must be <= hsv_upper for every channel")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    if glare_profile is None:
        return mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    repaired = mask.copy()
    for contour in contours:
        if not config["min_area_px"] <= cv2.contourArea(contour) <= config["max_area_px"]:
            continue
        hull = cv2.convexHull(contour)
        region = np.zeros(mask.shape, np.uint8)
        cv2.fillConvexPoly(region, hull, 255)
        pixels = region > 0
        pixel_count = np.count_nonzero(pixels)
        if pixel_count == 0 or np.count_nonzero(mask[pixels]) / pixel_count < glare_profile["min_blue_fraction"]:
            continue
        near = cv2.dilate(region, np.ones((11, 11), np.uint8))
        far = cv2.dilate(region, np.ones((41, 41), np.uint8))
        ring = (far > 0) & (near == 0)
        if not np.any(ring):
            continue
        cutoff = min(glare_profile["max_value"],
                     float(np.median(hsv[:, :, 2][ring]))-glare_profile["background_value_gap"])
        neutral_dark = ((hsv[:, :, 1] <= glare_profile["max_saturation"]) &
                        (hsv[:, :, 2] <= cutoff) & pixels)
        component = cv2.bitwise_and(mask, region)
        component[neutral_dark] = 255
        component = cv2.morphologyEx(component, cv2.MORPH_CLOSE, kernel)
        # Reconstruction cannot enlarge the measured blue convex hull.
        repaired |= cv2.bitwise_and(component, region)
    return repaired


def detect(frame, config, glare_profile=None):
    h, w = frame.shape[:2]
    roi = config.get("roi_xywh") or [0, 0, w, h]
    if not isinstance(roi, (list, tuple)) or len(roi) != 4:
        raise ValueError("roi_xywh must be a 4-element [x, y, w, h] list")
    x, y, rw, rh = map(int, roi)
    if x < 0 or y < 0 or rw <= 0 or rh <= 0 or x + rw > w or y + rh > h:
        raise ValueError("Detection ROI is outside the image")
    if glare_profile is None:
        mask = blue_mask(frame[y:y+rh, x:x+rw], config)
    else:
        # Keep the same glare background ring as the full-image pose fitter.
        mask = blue_mask(frame, config, glare_profile)[y:y+rh, x:x+rw]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not config["min_area_px"] <= area <= config["max_area_px"]:
            continue
        bx, by, bw, bh = cv2.boundingRect(contour)
        # Reject clipped objects; a clipped centre is not usable for grasp planning.
        if bx <= 1 or by <= 1 or bx + bw >= rw - 1 or by + bh >= rh - 1:
            continue
        rect = cv2.minAreaRect(contour)
        a, b = rect[1]
        if min(a, b) <= 0:
            continue
        if max(a, b) / min(a, b) > config["max_aspect_ratio"]:
            continue
        if area / (a * b) < config["min_rectangular_fill"]:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        centre = [moments["m10"] / moments["m00"] + x,
                  moments["m01"] / moments["m00"] + y]
        box = cv2.boxPoints(rect) + np.array([x, y])
        edge = box[1] - box[0]
        candidates.append({"centre_px": centre, "area_px": float(area),
                           "image_angle_deg": math.degrees(math.atan2(edge[1], edge[0])),
                           "box_px": box.tolist()})
    candidates.sort(key=lambda item: item["area_px"], reverse=True)
    status = "single_candidate" if len(candidates) == 1 else (
        "no_candidate" if not candidates else "ambiguous_multiple_candidates")
    return {"status": status, "image_size_wh": [w, h],
            "candidates": candidates, "robot_motion_ready": False}


def overlay(frame, result):
    rendered = frame.copy()
    for index, candidate in enumerate(result["candidates"], 1):
        box = np.rint(candidate["box_px"]).astype(np.int32)
        centre = tuple(round(v) for v in candidate["centre_px"])
        cv2.drawContours(rendered, [box], 0, (0, 255, 255), 3)
        cv2.drawMarker(rendered, centre, (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.putText(rendered, str(index), (centre[0]+16, centre[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.rectangle(rendered, (0, 0), (rendered.shape[1], 80), (0, 0, 0), -1)
    cv2.putText(rendered, result["status"], (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(rendered, "Detection setup only - no robot movement", (15, 62),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return rendered


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config.json")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--glare-repair", action="store_true",
                        help="Apply the same bounded dark/glare recovery used by preview/pick")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "captures")
    args = parser.parse_args()
    if args.live and (args.image or args.headless):
        parser.error("--live requires a camera and a visible window")
    config = json.loads(args.config.read_text(encoding="utf-8-sig"))
    vision = config.get("vision", config)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    glare_profile = DEFAULT_GLARE_PROFILE if args.glare_repair else None
    camera = None
    try:
        if not args.image:
            placeholders = {"camera_ip": "CAMERA_IP", "camera_username": "CAMERA_USER",
                            "camera_password": "CAMERA_PASSWORD"}
            missing = [k for k, v in placeholders.items()
                       if not str(config.get(k, "")).strip() or str(config.get(k)) == v]
            if missing:
                raise SystemExit("Set " + ", ".join(missing) + " in " + str(args.config))
            camera = HTTPCamera(config["camera_ip"], config["camera_username"],
                                str(config["camera_password"]))
        if not args.headless:
            cv2.namedWindow("Blue cube detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Blue cube detection", 1100, 620)
        while True:
            capture_started = time.time()
            if args.image:
                frame = _read_image(args.image)
            else:
                frame = cv2.imdecode(np.frombuffer(camera.capture(), np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise RuntimeError("Image could not be decoded")
            result = detect(frame, vision, glare_profile=glare_profile)
            result["capture_started_unix"] = capture_started
            result["capture_completed_unix"] = time.time()
            result["source"] = str(args.image) if args.image else "fresh_http_capture"
            rendered = overlay(frame, result)
            # Latest captures are for inspection only; future picking must capture again.
            for name, data in [("latest_frame.png", frame), ("latest_overlay.png", rendered)]:
                _write_image(args.output_dir / name, data)
            (args.output_dir / "latest_detection.json").write_text(
                json.dumps(result, indent=2), encoding="utf-8")
            print(result["status"], [c["centre_px"] for c in result["candidates"]], flush=True)
            if args.headless:
                break
            cv2.imshow("Blue cube detection", rendered)
            key = cv2.waitKey(1 if args.live else 0) & 0xFF
            if not args.live or key in (27, ord("q"), ord("Q")):
                break
            if cv2.getWindowProperty("Blue cube detection", cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        if camera:
            camera.close()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
