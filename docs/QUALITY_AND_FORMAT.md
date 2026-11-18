# Calibration format and quality

Schema: `cobotta.eye_in_hand.v2`. JSON lengths are millimetres and reported angles
are degrees unless a field explicitly says radians. A 4 x 4 `T_A_B` maps points in
frame B into frame A. Flange means the physical J6 mounting frame. Controller
Tool 0 and RoboDK's model flange can instead reference the stock fingertip location;
the reference checks must resolve those offsets before collecting observations.
Base means zero-offset Work 0.

| Field | Meaning |
| --- | --- |
| `board` | Exact square/marker dimensions, dictionary, layout, marker IDs, PDF name and SHA-256 |
| `intrinsics.K` | 3 x 3 camera matrix |
| `intrinsics.distortion_opencv_k1_k2_p1_p2_k3` | Five Brown distortion coefficients |
| `hand_eye.T_mount_camera` | Optical camera frame into DENSO ROS J5 mounting frame |
| `hand_eye.mounting_link`, `mount_frame` | 5 and `DENSO_ROS_J5`; camera does not rotate with J6 |
| `table.T_base_board` | Printed board frame into base frame |
| `table.point_base_mm`, `normal_base` | Table plane; board backing thickness has been subtracted |
| `grasp_tcp.T_flange_grasp` | Geometry-derived grasp frame into flange frame; not estimated from board images |
| `quality` | Fit errors, held-out errors, coverage, excitation and pass/fail |
| `binding` | Robot, saved VIEW, frame definitions and vision/geometry fingerprints |
| `binding.controller_reference` | Verified conversion from the physical flange to the controller reference |
| `workspace` | Paper corners from calibrated pattern plus confirmed print registration, or four visible image-click corners mapped into the table plane |

The motion observations supply both `T_base_flange` and `T_base_camera_mount`.
The latter is calculated from controller joints through J5 using DENSO's published
kinematics. The stationary-board relationship is
`T_base_board = T_base_camera_mount * T_mount_camera * T_camera_board`.
Intrinsics are fitted from training observations, then OpenCV's Park hand-eye method
estimates the camera-to-J5 transform. Separate observations check the predicted
board image and ray/plane intersections without refitting the accepted transforms.

Default gates: at least 16 fit and 6 check views; fit RMS <= 0.8 pixels;
maximum per-check RMS <= 3 pixels; maximum check-corner plane error <= 1.5 mm;
maximum board-angle discrepancy <= 0.7 degrees. Image coverage must span at least
85% horizontally and 80% vertically, with a convex-hull area of at least 60% of the
image. Rotation and translation excitation checks reject insufficiently varied views,
and at least one fit view must tilt the board 15 degrees or more out of plane
relative to the first fit view; in-plane roll does not count. The collection now
measures that tilt while it plans, so a pose that cannot provide it stops the run
before the sweep instead of after it.
These are software acceptance thresholds, not demonstrated accuracy of this setup.

Accepted images still require at least 12 identified ChArUco corners spanning
multiple rows/columns with a hull area of at least 8,000 pixels squared. Partial
boards are allowed; four collinear corners cannot constrain calibration. A bounded
image-directed framing can acquire the initial pose after explicit confirmation,
through the same planner. Recorded robot/image response only proposes poses; it
is never used as an accepted intrinsic/hand-eye/table calibration. Fresh response
checks and predicted complete-marker visibility precede collected views. The
operator-confirmed measured rectangle replaces the temporary disk after paper
direction confirmation. This does not widen controller limits or relax any final
image/quality gate. Acquisition images are excluded from fit/check data.

Current partial-board observations use OpenCV 4.5.5 `minMarkers=1`, which retains
image-based subpixel refinement while permitting a corner supported by one visible
adjacent marker. Corners within 12 pixels of the sensor boundary are discarded.
The prediction requires at least one adjacent marker inside its 48-pixel margin.
This makes edge coverage possible without changing final error/count/coverage gates.
The reachable pool's predicted coverage is checked before collection; final
acceptance still uses the captured corners and heldout validation, not forecasts.

The `printed-page` workspace method uses the hash-locked supplied Letter layout
and the fitted board pose only after the operator confirms the actual untrimmed,
unshifted Actual Size print. Off-image edges are registered under that confirmed
assumption; they are not independently measured. `image-corners` retains the four
visible-corner method. The saved data records which method was used.

Metric errors are consistency errors against the printed target and robot pose
readings. They do not measure absolute grasp accuracy or correct inaccurate printing,
backlash, finger geometry errors or camera motion after calibration.

The picker consumes this exact schema, verifies its saved hash and board definition,
fits a known-size cube silhouette through the camera model, and applies the separate
grasp transform. Visibility alone does not guarantee reachability, a clear approach,
a valid grip or a collision-free return with the held cube.

The entire detected cube outline must lie inside `calibrated_image_hull_px`,
preventing use outside the training image region. Accepted files are saved to
unique `calibrations/*_calibration_v2.json` paths before the atomic config-reference
update; an unsuccessful update does not overwrite the previous accepted file.

Collision checks use the station model and generated table, gripper and factory
camera enclosures at sampled joint positions. The camera enclosure retains the
stock CAD camera, bracket, fasteners and cable geometry, removes the wrist casing,
and encloses subdivided local triangle groups with 2 mm allowance and an 18.5 mm
sweep to cover both documented factory mounting positions. Triangles are split
to <=4 mm axis spans before 4 mm grouping, within isolated original 8 mm groups,
to avoid the old long-triangle boxes filling empty mounting-end space. All
retained CAD is still enclosed; see `HARDWARE_GEOMETRY_NOTES.md`. It follows J5; camera/link-4,
camera/link-6, camera/gripper and camera/payload checks remain enabled. Only the
camera's intended attachment to link 5 is excluded. The asset hash is checked
before calibration or picking. They do not cover unmodelled objects,
people or cables outside the supplied enclosure. They are not continuous certified
collision avoidance. Start states rejected by the controller are not bypassed.

The gripper body uses a convex enclosure of the official stock CAD, expanded by
the full 2 mm allowance. This replaces false occupied corners of its old AABB;
the fingers keep their opening sweep. Both prebuilt meshes are hash-checked and
require the verified 2 mm margin setting. Offline containment and pair geometry
inspection does not validate a full RoboDK route or physical clearance.
