# Configuration

All settings live in three JSON files at the project root. Keep the file names
and keep them next to the entry script.

## 1. `pick_config.json` — robot and pick settings

| Field | Meaning |
| --- | --- |
| `robot_ip` | b-CAP address of the COBOTTA controller, e.g. `192.0.2.10` |
| `observation` | Saved VIEW. Leave `pose` and `joints` as `null`; `set-view` fills them |
| `approach_clearance_mm` | Height above the grasp pose for the approach move |
| `lift_mm` | Lift distance after gripping |
| `speed_percent` | Must stay `100`; the program refuses other values |

After `calibrate` succeeds, the program adds a `calibration_v2` entry that
points at the accepted file in `calibrations/`. Do not edit it by hand.

## 2. `vision_config.json` — camera and colour settings

| Field | Meaning |
| --- | --- |
| `camera_ip` | Camera address, e.g. `192.0.2.30` |
| `camera_username` | HTTP Digest user name |
| `hsv_lower`, `hsv_upper` | Blue-cube HSV range |
| `roi_xywh` | Optional image region, or `null` |
| `min_area_px`, `max_area_px`, `max_aspect_ratio`, `min_rectangular_fill` | Cube detection limits |

The camera **password is never stored in the repository**. Set it as an
environment variable before starting RoboDK:

    setx COBOTTA_CAMERA_PASSWORD "your-camera-password"

or, for the current PowerShell session only:

    $env:COBOTTA_CAMERA_PASSWORD = "your-camera-password"

The program stops with a clear message if the variable is missing.

## 3. `automatic_setup.json` — verified setup (do not edit casually)

This file holds the checked robot model, gripper/board dimensions, quality
limits, motion settings and collision asset hashes. Leave it unchanged unless
the robot, gripper, camera mount or printed board actually changes. The program
verifies the board fingerprint, the collision asset hashes and the 2 mm margin,
and stops if they do not match.

## 4. Project folder for RoboDK temporary copies

RoboDK may execute the script from `%TEMP%\Prog1.py`. In that case the script
cannot infer the project folder from its own location. Set:

    setx COBOTTA_PROJECT_DIR "C:\path\to\cobottaPickAndLift"

or edit `PROJECT_DIRECTORY` near the top of `cobotta_calibrate_pick.py` and
`pick_blue_cube.py`. An override must point at a complete folder.

## Keep your site values out of commits

The checked-in `pick_config.json` is a template. `set-view` and `calibrate`
write your VIEW and calibration binding into the same file. If you contribute
back to this repository, either commit the template before running the robot or
restore it afterwards, and keep your local `robot_ip` out of commits. Your
accepted calibration files in `calibrations/` are already ignored by
`.gitignore`.

## First-run checklist

1. Edit `pick_config.json` and `vision_config.json`.
2. Set `COBOTTA_CAMERA_PASSWORD` (and `COBOTTA_PROJECT_DIR` if needed).
3. Load the script in RoboDK and run `status` — it reads the controller
   without motion and prints the current state, frames and speed report.
4. Run `set-view` to save the working pose as VIEW.
5. Run `calibrate` with the ChArUco board flat and visible.
6. Run `preview`, then `pick`.
