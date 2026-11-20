# cobottaPickAndLift

Eye-in-hand calibration and pick-and-lift for a DENSO COBOTTA with a wrist-mounted Canon N10-W02 camera.

RoboDK is used offline for kinematics checks and collision-checked path planning. Physical motion runs directly over DENSO b-CAP (TCP 5007) after explicit operator confirmations.

## Overview

- **Calibration (`calibrate`)**: Measures a local robot-to-image response around the saved `VIEW` pose, frames the 7x5 ChArUco board, collects 16+ fit and 6+ held-out check views along bounded segments, fits camera intrinsics, J5 hand-eye transform, and table plane, and registers the workspace paper polygon.
- **Preview & Pick (`preview`, `pick`)**: Detects a single blue 25 mm cube on the calibrated table, verifies its 3D top-face pose and paper clearance, plans approach, vertical descent, lift, and held return to `VIEW`, and (in `pick`) executes the grasp and lift.
- **Diagnostics (`status`, `set-view`, `detect_blue.py`)**: Inspect controller state/overrides, save a new `VIEW` pose without arm motion, or tune HSV cube segmentation offline/live.

## Documentation

- [Installation guide](docs/installation_guide.md) — dependencies, `config.json` setup, RoboDK setup, and troubleshooting
- [Operating manual](docs/OPERATING.md) — workflow modes, safety interlocks, calibration steps, picking, and session outputs
- [Calibration format and quality](docs/QUALITY_AND_FORMAT.md) — `cobotta.eye_in_hand.v2` schema, coordinate conventions, and acceptance gates
- [Hardware geometry notes](docs/HARDWARE_GEOMETRY_NOTES.md) — J5 camera mount, gripper/camera collision envelopes, and CAD references
- [Third-party notices](docs/THIRD_PARTY_NOTICES.md) — bundled b-CAP client, CAD envelopes, and ChArUco assets

## Repository layout

| Path | Purpose |
| --- | --- |
| `cobotta_calibrate_pick.py` | Main RoboDK Python entry point (`status`, `set-view`, `calibrate`, `preview`, `pick`, `board`, `instructions`) |
| `bcapclient.py`, `variant.py`, `orinexception.py` | DENSO b-CAP TCP transport |
| `http_camera.py`, `detect_blue.py` | Canon WebView HTTP snapshot client and standalone blue-cube detector |
| `automatic_setup.json` | Hardware dimensions, gripper/planning limits, quality gates, and asset hashes |
| `config.json` | Tracked template with placeholders; fill in locally before running |
| `collision_assets/` | Hash-checked STL envelopes for the J5 camera assembly and J6 gripper body |
| `charuco-7x5.pdf`, `charuco-7x5-letter.pdf` | Reference ChArUco board and US Letter printable sheet |
| `calibrations/` | Accepted `*_calibration_v2.json` files (ignored by git) |
| `sessions/` | Per-run logs, images, routes, and diagnostics (ignored by git) |

## License

MIT — see [LICENSE](LICENSE) and [docs/THIRD_PARTY_NOTICES.md](docs/THIRD_PARTY_NOTICES.md).
