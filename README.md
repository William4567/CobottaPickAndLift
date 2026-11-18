# cobottaPickAndLift

Eye-in-hand calibration and pick-and-lift for a **DENSO COBOTTA** with a
wrist-mounted Canon N10-W02 camera.

RoboDK is used for simulated planning and collision checking only. The physical
arm is driven over DENSO b-CAP after an explicit operator confirmation.

> **Safety first.** This is production code for a real robot. Read
> [docs/OPERATING.md](docs/OPERATING.md) before the first motion. Keep the
> commanded speed/acceleration/deceleration at 100%, keep the physical stop
> within reach and the swept area clear. The program never retries, recovers,
> releases or resets faults automatically.

## Requirements

- Windows and RoboDK with a station containing the COBOTTA.
- RoboDK's embedded Python: `C:\RoboDK\Python-Embedded\python.exe`.
- `numpy` 1.25.x and `opencv-contrib-python` **4.5.5** with `aruco`.
- DENSO COBOTTA reachable over b-CAP (TCP 5007).
- Canon N10-W02 camera reachable over HTTP Digest (1920 x 1080).

No administrator rights are needed: the program and its dependencies can live
entirely under your user profile.

## Quick start

1. Clone the repository, e.g. to
   `C:\Users\<you>\Documents\cobottaPickAndLift`.
2. Install the Python packages **inside the repository** (no admin):
   ```powershell
   "C:\RoboDK\Python-Embedded\python.exe" -m pip install --target "C:\Users\<you>\Documents\cobottaPickAndLift\vendor" "numpy==1.25.2" "opencv-contrib-python==4.5.5.64"
   ```
   Skip this if RoboDK's Python already has OpenCV 4.5.5 and numpy 1.25.
3. In RoboDK: **File > Open** `cobotta_calibrate_pick.py` so it becomes a
   Python item, with `C:\RoboDK\Python-Embedded\python.exe` as interpreter.
4. Run the item once. It creates **one config file**:
   `C:\Users\<you>\AppData\Roaming\cobottaPickAndLift\config.json`
   Edit `project_directory`, `robot_ip`, `camera_ip`, `camera_username` and
   `camera_password`, then run again.
5. Choose a mode from the dropdown: `status`, `set-view`, `calibrate`,
   `preview`, `pick` (plus `board` and `instructions`).

Full instructions, field descriptions and troubleshooting:
**[guide.md](guide.md)**.

## Files

| File | Purpose |
| --- | --- |
| `cobotta_calibrate_pick.py` | Main entry point (RoboDK Python item) |
| `bcapclient.py`, `variant.py`, `orinexception.py` | DENSO b-CAP transport |
| `http_camera.py`, `detect_blue.py` | Camera snapshot and blue-cube detection |
| `automatic_setup.json` | Verified robot/gripper/board/quality/collision setup |
| `config.example.json` | Reference copy of the generated config file |
| `guide.md` | Setup, configuration and troubleshooting |
| `collision_assets/` | Hash-locked camera and gripper envelopes |
| `charuco-7x5.pdf`, `charuco-7x5-letter.pdf` | Calibration targets |
| `calibrations/` | Accepted calibrations written by the program |
| `sessions/` | Per-run evidence written by the program (not in the repository) |
| `docs/` | Operating, quality, environment and hardware notes |

## Outputs

Each run creates `sessions/<timestamp>_<mode>/` with images, observations,
routes, `timings.json`, `speed_report.json` and, on a stop, `failure.json`.
Accepted calibrations are written to `calibrations/` and bound in the config
file.

## License

MIT — see [LICENSE](LICENSE). Third-party components and sources are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
