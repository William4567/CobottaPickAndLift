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
- `opencv-contrib-python` **4.5.5** (other versions are rejected on purpose).
- DENSO COBOTTA reachable over b-CAP (TCP 5007).
- Canon N10-W02 camera reachable over HTTP Digest (1920 x 1080).
- The three JSON files, `collision_assets/` and the two ChArUco PDFs must stay
  in the project root.

See [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md) for the tested versions.

## Install

1. Clone the repository, e.g. to `C:\path\to\cobottaPickAndLift`.
2. In RoboDK, open the COBOTTA station and load `cobotta_calibrate_pick.py`
   with **File > Open** so it appears as a Python item. Select
   `C:\RoboDK\Python-Embedded\python.exe` as its interpreter.
3. Configure the robot and camera — see
   [docs/CONFIGURATION.md](docs/CONFIGURATION.md). At minimum, set:
   - `pick_config.json` → `robot_ip`
   - `vision_config.json` → `camera_ip`, `camera_username`
   - the `COBOTTA_CAMERA_PASSWORD` environment variable
4. If RoboDK runs the script from a temporary copy, set
   `COBOTTA_PROJECT_DIR` to this folder (or edit `PROJECT_DIRECTORY` at the top
   of the entry script).
5. Run the Python item and choose a mode from the dropdown:
   - `set-view` — save the current robot pose as VIEW (no motion)
   - `calibrate` — collect a fresh eye-in-hand calibration
   - `preview` — locate the cube and plan, without motion
   - `pick` — physically pick and lift the cube
   - `status`, `board`, `instructions` — diagnostics and helpers

The first calibration needs a saved VIEW: `calibrate` stops with a clear
message if `set-view` has not been run yet.

## What is in the folder

| File | Purpose |
| --- | --- |
| `cobotta_calibrate_pick.py` | Main entry point (RoboDK Python item) |
| `pick_blue_cube.py` | Compatibility launcher; delegates to the entry point |
| `bcapclient.py`, `variant.py`, `orinexception.py` | DENSO b-CAP transport |
| `http_camera.py`, `detect_blue.py` | Camera snapshot and blue-cube detection |
| `automatic_setup.json` | Verified robot/gripper/board/quality/collision setup |
| `pick_config.json`, `vision_config.json` | Site settings you edit (IPs, clearance) |
| `collision_assets/` | Hash-locked camera and gripper envelopes |
| `calibrations/` | Accepted calibrations written by the program |
| `sessions/` | Per-run evidence written by the program (not in the repository) |
| `docs/` | Operating, configuration, quality and hardware notes |

## Outputs

Each run creates `sessions/<timestamp>_<mode>/` with images, observations,
routes, `timings.json`, `speed_report.json` and, on a stop, `failure.json`.
Accepted calibrations are written to `calibrations/` and bound in
`pick_config.json`.

## License

MIT — see [LICENSE](LICENSE). Third-party components and sources are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
