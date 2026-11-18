# COBOTTA Python: CalibrX board edition

Current revision: `2026-11-13-fast-planning-r8`. This remains the existing
RoboDK Python workflow. The 30 October real run reached the new VIEW, found all
17 markers and 24 ChArUco corners, then stopped on the old historical-response
prerequisite. The new revision obtains a bounded response at the current pose
instead of loading old coefficients. Both framing and collection use that fresh,
independently checked response. It is provisional guidance, not calibration.

The latest real run passed the response probes (0.07 board mm independent error),
paper-direction confirmation and framing skip, then stopped before collection:
its candidate pool forecast only 82.6% image width against the required 85%.
The candidate list now also samples +/-50 board mm horizontally, across the same
four heights, adding eight checked proposals. The new targets still require live
IK/workspace/collision/route approval and fresh-image verification. Required width,
marker margins and all final calibration limits remain unchanged.

The subsequent run passed pool preparation, then the second collection transfer
(roughly 174 mm between angular extremes) disagreed with local response by 21.53
board mm. Both images retained sufficient corners. Collection now prefers nearer
useful poses and divides routes into <=8 mm flange/J5 translation and <=2 degree
flange/J5 rotation or joint-change segments. Each stops for a fresh accepted image
and <=2 board mm response check. Only a verified image pair can refresh local
guidance; unstable updates stop. The next image checks the new estimate. Transit
images do not enter calibration fit/check sets. Final quality limits remain.

Your current VIEW is already saved: do not run set-view again for this verification.
The stopped collection pose is outside the 35 mm startup region. A separate
NEW-run dialog can reaffirm the last matching rectangle for a freshly checked
return to the existing VIEW, provided physical paper/directions are unchanged.
Cancel if they changed. The 35 mm probe region is restored at VIEW and fresh
response acquisition starts; the failed route is never resumed.
Source inspection and syntax compilation only were performed for this revision;
no simulation, synthetic/mock suites, entry-point execution or hardware connection.
Bounded collection/local updates/startup return are UNVERIFIED until you run them. Earlier offline checks
are historical and do not demonstrate successful real calibration or picking.
All existing calibration acceptance limits, collision margins and live-setting
policies remain.

## Latest progress and runtime improvements

The latest real run passed independent checking at0.17 board mm and114 bounded
collection response checks (maximum1.316 mm). It reached9 fit/3 check before the
camera HTTP read timed out after reported computer sleep. The61.7% display meant
sensor-area coverage, NOT workflow completion. At least16 fit/6 check and all
existing width/height/area/excitation/error gates are required. Partial files remain.

The new revision avoids redundant controller handle acquisition/release while
reading every value fresh, and reuses pure model FK calculations locally. It keeps
all stability/stop/image/collision/controller/quality checks and the8 mm/2 degree
segment limits. Progress now prints the required counts/spans/area separately.
This runtime improvement has not yet been measured on the real robot.

Calibration temporarily requests Windows idle sleep/display prevention, restoring
the previous request at completion, cancellation or error. It changes no power-plan
settings. Keep the laptop lid open and avoid manual Sleep/power actions. An
interruption still stops; the code does not reconnect or automatically resume.

Reload the new revision and run calibrate with the saved VIEW/board/optics fixed.
Follow the usual confirmations, check for the keep-awake message, and continue to
the first stop or completed calibration. Saved per-checkpoint timings allow the
next session to verify actual speed. Do not start picking for this verification.

API references: [DENSO b-CAP variable lifecycle](https://support.densorobotics.com/en/support/solutions/articles/60000698627-orin-2-programming-in-python-3-bcap-packets-),
[Windows keep-awake request](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-setthreadexecutionstate).

## Current-pose response and focused robot verification

At the latest saved VIEW, all ten axis-probe preflights passed, but the single
combined holdout [+3,-3,+3] mm / [+1,-1] degrees failed IK. The program now chooses
the independent check from four fixed combined candidates, all within the same
limits, using full outward/return preflight before any probe moves. The confirmation
shows the selected vector. No candidate passing means a stop with no probes.
`independent_check_candidates.json` records decisions. The holdout remains excluded
from fitting and must pass the unchanged 2 board mm independent verification.

Reload the updated Python item and select `calibrate`; verify the printed revision.
Keep the board fixed and clear, with optics unchanged. After the existing startup
confirmations and VIEW capture, a separate dialog describes ten small out-and-back
probes: base XYZ +/-5 mm and world Rx/Ry +/-2 degrees, one axis at a time. An
eleventh independent check uses the selected XYZ/RxRy vector displayed in the
motion confirmation (each XYZ component +/-3 mm, each Rx/Ry component +/-1 degree).
All direct outbound/return paths must preflight before that confirmation. No RRT
detours or larger probes are used. Probes stay inside the existing temporary
35 mm region, with additional sampled flange (8 mm/2.5 degrees), J5 mount
(15 mm/5 degrees) and per-joint (8 degrees) bounds from the reference pose.

Each valid fresh probe image permits a checked return; the return must reproduce
the original board image and J5 pose. A failed image/response/readiness check stops
without a return. The program fits only current-run measured poses/images, checks
fit conditioning and residuals, then checks the independent probe excluded from
fitting. Expect `Current-pose response verified` only after its return also passes.
No probe image contributes to the eventual camera-calibration fit/check sets.

Confirm the paper-direction arrows/measurements next. Adequate framing skips all
corrective framing; otherwise a separate dialog authorizes the existing correction
policy. The saved 30 October image has 10.81 mm centring error versus the unchanged
10 mm limit, so a small correction may still be needed despite all corners being
detected. No historical VIEW is restored.

Confirm collection to exercise the collector in this same run. Let it reach
acceptance or the first stop. Finish the normal paper-boundary/return procedure
if calibration passes, but do not proceed to picking for this verification.
Tell the agent when it finishes/stops; the newest local session contains the
images, probe plan, `framing_response_model.json`, `response_validation.json`,
collection records and any `failure.json`. No duplicate screenshot request is needed.

The entry point is `cobotta_calibrate_pick.py`. It uses the supplied
`charuco-7x5.pdf`: 7 x 5 squares, **25 mm squares**, **18 mm markers**,
`DICT_4X4_50`, IDs 0 through 16. The pattern is **175 x 125 mm**.
Print **`charuco-7x5-letter.pdf` on US Letter (8.5 x 11 inches)**. It contains the
same physical pattern translated onto the Letter page without resizing.

## Run from RoboDK

If the paper measurements refer to the robot's current pose rather than the old
saved VIEW, run **`set-view` first**. It reads the real controller joints/pose,
shows a fresh camera image, and asks you to confirm that this pose is the paper's
measurement reference. It saves a new VIEW without moving the arm or gripper.
The old configuration is backed up, the old calibration binding is removed, and
existing calibration files are preserved. Run `calibrate` separately afterward.

1. Keep the entry point, existing Python helpers and JSON files together in this folder.
   Open `pickandplace`, then load `cobotta_calibrate_pick.py` using RoboDK's File > Open
   so it appears as a Python item in the station tree. Use
   `C:\RoboDK\Python-Embedded\python.exe` as the Python interpreter.
   If the item already contains an older pasted copy of the code, replace that
   item's code with the updated file; editing the disk file does not update an
   old embedded copy automatically.
2. Run that Python item when ready. A dropdown lists the operations; select one
   and press OK (`Cancel` ends the run with no operation and no motion). If this
   Python build has no Tk, the previous typed dialog appears instead. Choose
   `board` to open the supplied sheet, `calibrate` to collect data, `preview`
   to inspect a proposed pick without movement, or `pick` to execute a fresh
   pick. `status` reads the controller without motion.
   RoboDK's planning mode is now set to Simulation by the program. This does not
   simulate physical b-CAP calibration/pick commands: those operations can move
   the real arm after the explicit motion confirmation.
3. Select US Letter paper and print at **Actual Size / 100%**, with Fit/Shrink disabled. Measure the
   printed 50 mm bar, a 25 mm square, and both overall pattern dimensions.
   Use a flat backing and enter the paper-plus-backing thickness when prompted.
4. Before collection, lock focus/zoom and keep 1920 x 1080 capture settings fixed.
   Remove objects covering the pattern and secure the board before acquisition.
   It must remain stationary throughout acquisition and all fit/check views.
   Partial views are allowed; each accepted image still needs at least 12 ChArUco
   corners across multiple rows/columns and an 8,000 pixel-squared hull. Do not
   shrink the print to make it fit. Framing now uses the fresh image and the
   independently verified robot/image response measured in this run. Confirm paper directions
   on a stationary snapshot before the full measured rectangle replaces the
   temporary 35 mm region. Lateral corrections come first. Camera settings are
   not changed. Lost/inconsistent image reference stops without an automatic return.
5. The supplied configuration records your confirmation that the Canon camera has
   never moved from its original factory mounting. The obsolete six-number camera
   box prompt is removed. Keep `collision_assets/` with this program: it contains
   the camera/bracket/cable enclosure extracted from the stock robot CAD, covering
   both factory mounting positions with 2 mm allowance. Its refined triangle
   grouping removes false filled corners near the gripper; collision checks
   between camera and gripper remain enabled. It follows wrist link 5;
   the gripper follows J6. Different hardware or cable routing needs revised geometry.
   The stock gripper body now uses a CAD convex envelope, retaining its full 2 mm
   allowance; fingers retain the opening-sweep box. Keep both collision STL files
   in `collision_assets/`. The prebuilt meshes require the configured 2 mm margin.
   No camera or grasp pose is taught manually. When motors are OFF, calibration
   and pick offer a startup motor-on confirmation and verify the ON state before
   continuing. This program does not reset faults or change other live settings.
6. After an accepted calibration, remove the board, place one blue cube on the same
   table and choose `preview` or `pick`. Preview requires the robot already at VIEW.
   Pick routes to VIEW, captures again, approaches, grips, lifts and returns to VIEW
   holding the cube. It does not place or release it. The descent and lift are
   executed as fewer, longer verified straight segments (up to 10 mm each) and
   motion/gripper state is polled every 50 ms, so a stop request is noticed
   sooner. Every merged segment is still sampled and collision-checked, and the
   commanded path is preflighted before motion. If the cube outline is
   outside the region covered by the calibration images, the stop reports the
   offending image point, how many pixels outside it is and the covered pixel
   region, so you can move the cube or recalibrate with coverage there.

Planning tries the validated IK branches for a clear straight approach, then a
validated two-leg route through the Cartesian midpoint, so a cube placed further
to one side is not immediately forced into the random route search. The console
prints which one connected, and the session keeps `planning_diagnostics.json`
with the reason each straight path was rejected. Every branch still passes the same model/workspace/collision check,
controller range check and FK tolerance, and every straight path is sampled
like any other route. If no branch has a clear straight path, the nearest valid
branch is routed exactly as before, with the existing RRT search. Expect
`Direct joint path is blocked for all ... IK solution(s): ...` in the console
when that happens; the reason names the check that rejected the straight path.
No route, collision, limit or tolerance was relaxed for this.

The collision pair between the stock gripper-body envelope and the factory
camera/bracket envelope is **disabled** at the operator's request. The operator
physically verified that the gripper rotates freely in both directions,
including while holding a cube. The supplied camera envelope deliberately
covers two documented factory mounting profiles plus the CAD cable loop with a
2 mm margin, which is why it reported interference from about 25 degrees of
wrist rotation. Every other camera check remains enabled: the finger opening
sweep, robot links 4 and 6, the table, the held payload, the target cube and
station objects; the 2 mm margin and all other pairs are unchanged. This is an
operator safety decision about the real hardware. If the camera bracket or its
cable routing is ever changed, that pair is no longer covered by the model and
must be re-checked before picking.

Planning is now faster and quieter. RoboDK rendering is switched off while the
planner is open, so the 3D view does not redraw every sampled pose (it is
restored when planning ends); collision checking uses item poses, not the
rendered view. The pure controller `OutRange` query is remembered for an
identical joint vector inside planning checks, while the check immediately
before every move is still live. A path is not re-checked live when the exact
same path was already checked against an identical collision scene: the console
then prints `... reusing that preflight ...`. The lift with the newly held cube
is always checked fresh because its scene did not exist at planning time. Keep
the station and the cell unchanged while a pick is planned; an external RoboDK
edit or a collision caused by an object outside the modelled scene is not
re-detected by a reused check.

Before saving an accepted calibration, choose how to locate the paper boundary.
`printed-page` uses the calibrated pattern and the exact supplied Letter PDF
layout, after explicit confirmation of an untrimmed, unshifted Actual Size print.
Its outside corners may be off-image. This is a confirmed registration, not an
independent measurement of unseen paper edges. Use `image-corners` to click four
visible outside paper corners instead; click around the perimeter, right-click
to undo. The camera is at the acquired calibration view during this step, then
returns to saved VIEW through the planner before activating the calibration.
Saved VIEW's gripper projection must be inside the resulting polygon.
The paper must remain in that location for picking. The grasp centre is constrained
to that polygon along pick routes; the cube footprint must stay 5 mm inside it.
The arm and camera can extend beyond the paper. The initial return to VIEW and
the confirmed response probes use the temporary 35 mm radius. After the probes,
mark two paper points at least 20 mm apart
along the page's long edge towards you. Confirm the displayed TOWARDS YOU and
YOUR RIGHT arrows and the existing measurements: 70 mm left, 140 mm right,
40 mm behind and 240 mm towards you from saved VIEW's gripper projection.
The rectangle then replaces the circle, with 5 mm inset; framing targets reserve
another 5 mm. The arrows use this run's verified image/robot response,
without guessing photo-left as a robot axis. Incorrect/uncertain arrows
must be cancelled. Controller joint limits and collision checks remain active.
After calibration, the fitted/confirmed paper polygon is also enforced for the
return to saved VIEW before the calibration is activated.

Command speed, acceleration and deceleration remain **100%**. Live external
overrides are read and reported, never rewritten; they can still limit effective speed.
Tool 0 retains its zero user offsets. Before motion, read-only controller J2P
queries at multiple hypothetical joint configurations are compared with RoboDK
flange kinematics. They must agree through one consistent rigid offset: either
the physical flange or the supported stock 60 mm fingertip reference. The detected
offset is reported and used to convert camera/grasp planning into physical flange
coordinates. An inconsistent or unsupported offset stops the program. No joint
offsets or controller frame definitions are changed. Camera calibration and the
geometry-derived grasp TCP remain separate. Recalibrate older data with this version.

Esc/Q in the image viewer, Ctrl+C, or a file named `STOP_COBOTTA` in this folder
requests a stop. Software/network stop handling cannot substitute for the physical
stop. An error does not trigger automatic return or release.

## Motor preparation before calibration or pick

Revision `2026-10-28-confirmed-motor-on` adds operator-confirmed startup motor-on
for calibration and executing picks. When `@SERVO_ON` is false, OK in the
**ENABLE ARM MOTOR POWER?** prompt authorizes one motor-on request. Cancel ends
the run without that request. If power is already ON, the request is skipped.
The later, separate calibration/pick motion confirmation is still required.

After confirmation the program checks stationary state, frames, errors, empty
gripper and joint limits again. It acquires arm authority with `TakeArm([0,1])`:
the Keep option preserves current internal speed and Tool/Work selections. It
sends `Motor([1,1])` (ON, without blocking for completion), then polls readiness
for up to 10 seconds. It must obtain two stationary ON samples before proceeding.
Arm movement commands still explicitly use SPEED/ACCEL/DECEL=100. External
overrides, faults, limits and controller definitions are not rewritten.

Readings and command stages are saved in the session's `motor_readiness.json`.
Rejection, timeout or controller errors stop the run without an automatic retry,
fault reset or motor-off command. Check the DENSO status/error before restarting.
Power loss later still stops execution without re-enabling or resuming a route.
Preview, status and set-view never request motor-on.

Before the motor-on confirmation, `@EXTSPEED`, `@EXTACCEL` and `@EXTDECEL` are
re-read. If any is below the 5% startup floor, the run stops with a ValueError
that names all three values and asks you to set the pendant
speed/acceleration override to 100% and restart; no motor-on request, gripper
action or motion has happened. The 2026-11-13 pick started at
1.0 / 0.01 / 0.01 and crawled. Values at or above the floor are accepted and
become this run's change-detection baseline; if any is below 100, the motion
confirmation additionally states that effective speed, acceleration and
deceleration are limited by the pendant and that changing it during the run
will stop the run. The program still never writes an external override.

Reload the updated Python item in RoboDK. The console must print
`COBOTTA Python revision: 2026-11-13-fast-planning-r8`. An older pasted station item
will continue to run the old code even when the disk file has changed.

Keep the updated `collision_assets/factory_camera_J5_envelope.stl` and its
manifest with the project. This revision requires the refined asset hash saved
in `automatic_setup.json`. The project folder has been updated together; an old
asset combined with a new Python item stops at the hash check. The gripper boxes,
collision pairs, sampled planning checks and arm settings remain unchanged.

This change was reviewed and syntax-checked offline only. No program, tests,
controller connection, motor preparation or movement was run during development.

## Board acquisition and framing

The factory camera is offset beside the gripper and follows J5. A page centred
under the gripper need not be centred in the camera image; rotating J6 alone does
not reposition the camera. Failed and partial images now show marker/corner counts,
the number of observed board rows/columns, a white image-centre cross and an
approximate yellow pattern outline/centring arrow. Its homography is a provisional
framing measurement, never an accepted camera calibration. A blue region on the
pattern blocks framing motion and asks for a clear-board stationary recapture.
Blue colour outside the pattern is not treated as a pattern obstruction.

The upward-first sweep and the old fixed calibration grid are removed. Framing
and collection now use the separately confirmed, bounded current-pose probes
described above. Historical sessions and coefficients are not loaded. The board
and optics must stay fixed during this run; repeat images at the reference J5
pose and a separate combined probe verify the provisional response before use.

After paper-direction and framing confirmations, the first two lateral steps
are limited to 5 mm each; later ones to 10 mm. The direction is recomputed from
each fresh image. Small aiming corrections are available if needed. Height is
only considered at centring error <=10 mm when the outline span exceeds the
existing 1.02 framing-size limit (not the old 0.93 trigger), in
increments up to 15 mm and at most 40 mm above VIEW. There are at most 18
corrections. Every route still passes the full planner/controller preflight.
The console and saved route explain XYZ/aim changes and predicted image error.
Lost reference, disagreement, worsening/stalled centring or an obstruction stops.

A minimally accepted off-centre patch no longer ends acquisition: the code seeks
at least 18 corners and centring within 10 board mm. It then prepares a pool of
distinct reachable targets with at least 14 predicted corners, each supported by
one complete adjacent marker inside the prediction margin, with enough proposed
count, sensor coverage and two-axis rotation variation.
It stops before a calibration sweep if that pool is inadequate. Selected targets
compensate for predicted image displacement; selection seeks pose variation and
image coverage. Every selected route is preflighted. If a new image fails the
original 12-corner gate or disagrees with its prediction, collection stops before
another move. These predictions are approximate and are not clearance or
calibration acceptance claims.

Fit/check roles are assigned before capture. All original final quality gates
remain. Acquisition images are not fit/check training data. Only fresh collected
fit/check images enter the final solution. Startup motor-on and 100% arm commands
are preserved. Saved VIEW, live camera settings and controller limits are unchanged.

## Calibration files and limitations

Successful calibration writes a unique `calibrations/*_calibration_v2.json` and
then atomically binds it in the single config file
(`%APPDATA%\cobottaPickAndLift\config.json`). If saving the config fails, the
previous calibration file/reference remains intact. Keep the `calibrations/`
folder with the project. Picking also requires the cube outline to be inside
the image region covered by calibration observations.
Each session retains captures, corner overlays, observations, routes and quality
results under `sessions/`. Existing accepted calibration files are backed up before
replacement. Failed quality checks do not activate a new calibration.

Picking rejects calibrations whose board definition differs from this CalibrX
sheet. **Recalibrate after switching sheets**; changing the dimensions does not
convert previously collected calibration data. The board-scale confirmation and
backing thickness are requested again for the new board definition.

This revision also requires fresh calibration because the factory camera is mounted
on J5, before the rotating J6 flange. Old flange-based camera calibrations are not
loaded. Camera intrinsics and the J5-to-camera transform are estimated from new
observations; neither is replaced by a factory optical transform.

The supplied code has not been run or tested on the robot. The PDF was read and
its marker IDs decoded offline; this is not validation of calibration or picking.
No development robot connection, movement, motor enable, speed change, fault reset
or CALSET was performed. If the controller reports an out-of-range start such as
J6, the program stops. Automatic inward recovery is not verified for this controller
and is not implemented as an undocumented motion attempt.

See `QUALITY_AND_FORMAT.md` for the data convention and acceptance criteria.

## RoboDK temporary-script imports

RoboDK may execute Python items from a temporary path such as
`%TEMP%\Prog1.py`. The entry point now resolves the
complete project folder before importing `bcapclient`, and uses that folder for
all JSON, PDF and session paths. It prints the selected folder when started.
Keep the existing helpers in the project folder; they do not need installing
into RoboDK's site-packages or copying into Temp.

The project folder is the `project_directory` value in
`%APPDATA%\cobottaPickAndLift\config.json`. If you move the clone, update that
value; no environment variable is needed.
