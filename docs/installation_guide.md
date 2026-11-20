# Installation Guide

## Requirements

- Windows with RoboDK and a station containing the DENSO COBOTTA model (`pickandplace`).
- RoboDK embedded Python (`C:\RoboDK\Python-Embedded\python.exe`).
- `numpy` 1.25.x and `opencv-contrib-python` **4.5.5** (must include `cv2.aruco`). Other OpenCV versions are rejected because ChArUco corner ID and coordinate conventions differ across versions.
- DENSO COBOTTA controller reachable over b-CAP (TCP port 5007).
- Canon N10-W02 wrist camera reachable over HTTP (1920 x 1080 JPEG via `image.cgi`, Basic or Digest auth).
- [`charuco-7x5-letter.pdf`](../charuco-7x5-letter.pdf) printed on US Letter (8.5 x 11 in) at **Actual Size / 100%** (Fit/Shrink disabled).

## 1. Clone the repository

Clone the repository under your user profile (for example `C:\Users\<you>\Documents\cobottaPickAndLift`):

```powershell
git clone https://github.com/William4567/CobottaPickAndLift.git
```

## 2. Install Python dependencies

Skip this step if `C:\RoboDK\Python-Embedded\python.exe` already has `numpy` 1.25.x and `opencv-contrib-python` 4.5.5 installed.

To install the pinned packages locally inside the repository without modifying `C:\RoboDK`:

```powershell
"C:\RoboDK\Python-Embedded\python.exe" -m pip install --target "C:\Users\<you>\Documents\cobottaPickAndLift\vendor" "numpy==1.25.2" "opencv-contrib-python==4.5.5.64"
```

`cobotta_calibrate_pick.py` prepends `vendor\` to `sys.path` at startup.

## 3. Configure `config.json`

`config.json` is tracked with placeholder values so every clone starts with a valid template. Edit your local copy and fill in the four connection fields:

| Field | Value |
| --- | --- |
| `robot_ip` | COBOTTA controller IP address |
| `camera_ip` | Canon N10-W02 camera IP address |
| `camera_username` | Camera HTTP username |
| `camera_password` | Camera HTTP password |

Leave the remaining defaults unless your cell differs:

- `approach_clearance_mm` (`40`), `lift_mm` (`30`)
- `speed_percent` (must remain `100`)
- `observation` — populated automatically when you run `set-view`
- `vision` — HSV thresholds, ROI, and contour filters for blue-cube detection (tunable with `detect_blue.py`)

Do not commit your filled `config.json`. To reset placeholders before committing:

```powershell
git checkout -- config.json
```

## 4. Load and run in RoboDK

1. Open your COBOTTA station in RoboDK.
2. Choose **File > Open** and select `cobotta_calibrate_pick.py` so it is added as a Python item in the station tree.
3. Verify the item's interpreter is set to `C:\RoboDK\Python-Embedded\python.exe` (not system Python).
4. Run the Python item and choose an operation (`status`, `set-view`, `calibrate`, `preview`, `pick`, `board`, or `instructions`). See [OPERATING.md](OPERATING.md) for the workflow order and safety checks.

Whenever you update `cobotta_calibrate_pick.py` on disk (for example after `git pull`), re-open it in RoboDK via **File > Open** (or replace the item's script text). RoboDK executes a cached copy written to `%TEMP%` and will keep running old code until the item is reloaded.

You can also run the script directly from PowerShell (pass `--mode <operation>` to skip the menu):

```powershell
cd C:\Users\<you>\Documents\cobottaPickAndLift
"C:\RoboDK\Python-Embedded\python.exe" cobotta_calibrate_pick.py --mode status
```

`--help`, `board`, and `instructions` work before filling in `config.json` placeholders.

## Standalone cube-detection check (`detect_blue.py`)

To verify or tune `vision` HSV thresholds without touching the robot:

```powershell
"C:\RoboDK\Python-Embedded\python.exe" detect_blue.py --live
"C:\RoboDK\Python-Embedded\python.exe" detect_blue.py --image path\to\capture.jpg --headless
```

Press `Esc` or `Q` to close the live viewer. Captures and overlays are saved under `captures/`.

## Troubleshooting

- **`Fields to set: ...` / `Set camera_password ...`**: One or more placeholder values are still present in `config.json`. Fill in your real robot/camera fields.
- **`config.json is missing from the project folder`**: Restore it with `git checkout -- config.json`.
- **`COBOTTA project folder not found`**: Because RoboDK runs a temporary copy from `%TEMP%`, the script searches standard folders under your user profile (including `Documents`, `Desktop`, `Downloads`, and `OneDrive*` folders up to 4 levels deep). If your clone lives elsewhere, set `PROJECT_DIRECTORY` near the top of `cobotta_calibrate_pick.py` using a raw string:
  ```python
  PROJECT_DIRECTORY = r"C:\path\to\cobottaPickAndLift"
  ```
- **`Several copies of this project were found`**: More than one clone exists under your profile. Set `PROJECT_DIRECTORY` to the copy you want to use.
- **`This implementation uses OpenCV 4.5.5 ...`**: Install `opencv-contrib-python==4.5.5.64` into `vendor\` (step 2) and check that `C:\RoboDK\Python-Embedded\python.exe` is the active interpreter.
- **Asset SHA-256 mismatch on PDF/STL files**: Windows Git installations with `core.autocrlf=true` can corrupt binary files if checked out with an older `.gitattributes`. Run `git checkout HEAD -- charuco-7x5.pdf charuco-7x5-letter.pdf collision_assets/`.
