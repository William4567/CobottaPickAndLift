# Operating Manual

For installation, `config.json` setup, and RoboDK configuration, see [installation_guide.md](installation_guide.md).

## Workflow order

Run modes from `cobotta_calibrate_pick.py` in this order:

1. `status` — Read controller state, frames, external overrides, and `OutRange` without moving the robot or camera.
2. `set-view` — Preview the camera at the robot's current pose and save that pose as `VIEW` in `config.json`. Does not move the arm; opens the gripper only if it is not already open/empty, after confirmation.
3. `calibrate` — Measure local image response, frame the printed ChArUco sheet, collect fit/check views, solve intrinsics/hand-eye/table calibration, register the paper boundary, and save the accepted calibration.
4. `preview` — Capture at `VIEW`, detect and fit the cube, and compute a collision-checked pick plan without moving the arm; calls `ensure_gripper_open()` first (requiring `@SERVO_ON` and operator confirmation if the gripper is not already open/empty).
5. `pick` — Execute the full pick, lift, and return to `VIEW` holding the cube.

Two helper modes require no robot or camera connection: `board` opens `charuco-7x5-letter.pdf`, and `instructions` opens this file.

## Physical setup & safety checks

- **Calibration target**: Print [`charuco-7x5-letter.pdf`](../charuco-7x5-letter.pdf) on US Letter at **Actual Size / 100%** (7 x 5 squares, `25 mm` squares, `18 mm` markers, `DICT_4X4_50`, IDs `0..16`, `175 x 125 mm`). Measure the printed scale bar and keep the board flat on rigid backing (`board_thickness_mm` in `automatic_setup.json`, default `0.2 mm`).
- **Optics & vision settings**: Lock focus and zoom and keep `1920 x 1080` capture fixed before calibrating. Any focus, zoom, mount, or `config.json` `vision` setting change invalidates the saved calibration binding (`vision_settings_fingerprint`) and requires recalibration.
- **Frames & overrides**: `Tool 0` and `Work 0` must be active with zero user offsets. Arm `SPEED`, `ACCEL`, and `DECEL` commands are sent at `100%`. External pendant overrides (`@EXTSPEED`, `@EXTACCEL`, `@EXTDECEL`) are read-only; all three must be `>= 5%` at startup and must not change during a run.
- **Kinematics verification**: Before motion, `J2P` queries across multiple joint configurations are compared against RoboDK FK and DENSO's stock COBOTTA kinematics. Both must match through one consistent physical flange or `60 mm` stock fingertip offset.
- **Motor power**: If `@SERVO_ON` is `false` at the start of `calibrate` or `pick`, an explicit `ENABLE ARM MOTOR POWER?` dialog requests a single `Motor([1, 1])` enable and polls for a stationary `ON` state (`10 s` timeout). The program never resets faults, alters limits, runs `CALSET`, retries failed moves, or releases held objects automatically.

## Stopping a run

Keep the physical emergency stop accessible and the swept workspace clear. To request a software stop during execution:

- Press `Esc` or `Q` while the OpenCV image window has focus.
- Press `Ctrl+C` in the console.
- Create an empty file named `STOP_COBOTTA` in the repository root (the latch persists across restarts until you delete `STOP_COBOTTA` manually).

## Calibration (`set-view` and `calibrate`)

### Saving `VIEW` (`set-view`)
Position the arm where the board is visible inside your workspace paper, then run `set-view`. Paper boundary limits in `automatic_setup.json` (`reported_paper_limits_mm`: default `70 mm` left, `140 mm` right, `40 mm` behind, `240 mm` toward the user) are measured from this pose's gripper projection. Saving a new `VIEW` backs up `config.json` and clears any active `calibration_v2` binding.

### Calibration sequence (`calibrate`)
Calibration temporarily requests Windows idle-sleep prevention (`SetThreadExecutionState`) and restores the prior state on exit. Keep the laptop lid open and the board completely stationary throughout the run:

1. **Return to `VIEW`**: If the arm is away from `VIEW`, it plans a return within a temporary `35 mm` horizontal disk around `VIEW` (or re-confirms the previous run's `precalibration_workspace.json` if starting outside `35 mm`).
2. **Current-pose response probes**: After confirmation, runs 10 small single-axis out-and-back probes (`XYZ ±5 mm`, `Rx/Ry ±2 deg`; `Z_plus` omitted if unreachable by IK) plus 1 preflight-selected combined holdout (`±3 mm`, `±1 deg`), verifying repeatability and `<= 2.0` board mm prediction accuracy. These probes establish local guidance only and do not enter calibration training/check sets.
3. **Paper direction & workspace switch**: Displays `TOWARDS YOU` and `YOUR RIGHT` arrows on a fresh snapshot and replaces the `35 mm` disk with the `5 mm`-inset measured rectangle (`reported_paper_limits_mm`).
4. **Image-directed framing**: If centering error exceeds `10` board mm, fewer than `18` corners are visible, or the board outline exceeds `1.02` of the frame, up to 18 verified lateral/aiming/height steps center the board.
5. **Bounded collection**: Prepares a reachable candidate pool, verifies pool coverage/tilt bounds before moving, and collects `fit` and held-out `check` views along segments bounded to `<= 8 mm` / `2 deg` with fresh-image `<= 2` board mm response verification at every checkpoint.
6. **Fitting & registration**: Returns to the framing pose for `final_anchor_check`, fits intrinsics, J5 `T_mount_camera`, and table plane, evaluates acceptance gates (see [QUALITY_AND_FORMAT.md](QUALITY_AND_FORMAT.md)), registers the US Letter paper boundary (`printed-page`), returns to `VIEW`, writes `calibrations/<timestamp>_calibration_v2.json`, and binds it atomically in `config.json`.

## Preview and Pick (`preview` and `pick`)

Remove the calibration board and place one blue cube (`25 x 25 x 25 mm`) on the calibrated table inside the calibrated image hull and at least `5 mm` inside the registered paper boundary.

- **`preview`**: Calls `ensure_gripper_open()` at the current pose (requiring `@SERVO_ON` and operator confirmation if the gripper is not already open/empty) and requires the arm at `VIEW`. Captures two fresh images `0.3 s` apart, verifies cube localization repeatability (`< 0.8 mm`, `< 3 deg`), fits the cube top-face silhouette (`RMS <= 1.0 mm`, `IoU >= 0.85`), and solves a full pick/lift/return plan without moving the arm.
- **`pick`**: Opens and verifies the empty gripper at the current pose, moves to `VIEW`, captures and plans the pick, captures one final fresh image (`pick_final_fresh_check`) to confirm the cube did not shift during planning, then executes approach, vertical descent, `6 N` grip (`HandMoveH`), grip-width check (`25 ± 1.5 mm`), vertical lift (`lift_mm`), and held return to `VIEW`. The cube remains held at `VIEW`.

## Collision checking and planning notes

- Planning tries validated IK branches for a clear direct joint path first, then a checked Cartesian midpoint route, and finally joint-space RRT (`1600` iterations, `8 deg` step, `0.5 deg` collision sampling, `2.0 mm` margin).
- RoboDK 3D rendering is switched off during planning and restored when planning exits.
- The collision pair between the stock gripper-body hull (`stock_gripper_body_F_envelope.stl`) and factory camera envelope (`factory_camera_J5_envelope.stl`) is disabled because wrist clearance was physically verified; all other pairs (finger sweep, links, table, payload, target cube, and station objects) stay active. See [HARDWARE_GEOMETRY_NOTES.md](HARDWARE_GEOMETRY_NOTES.md).

## Session outputs

Session artifacts depend on the selected mode:

- `status` writes `sessions/<timestamp>_status.json`.
- `board` and `instructions` write nothing under `sessions/`.
- `set-view` writes `sessions/<timestamp>_set_view/` containing the preview capture, `new_view.json`, and (if the gripper had to be opened) `startup_gripper_open.json`.
- `calibrate`, `preview`, and `pick` write `sessions/<timestamp>_<mode>/` containing captured `.jpg` snapshots, `.png` overlays, and mode-specific outputs: `timings.json`, `speed_report.json`, `motor_readiness.json` on motion runs (`calibrate` and `pick`), `observations.json` and route JSONs on `calibrate`, `pick_plan.json` and `planning_diagnostics.json` on `preview` and `pick`, `startup_gripper_open.json` on `pick` (and on `calibrate` or `preview` if the gripper had to be opened), and `failure.json` on any stop or exception.
