# Factory camera geometry correction

## Gripper body correction — 29 October 2026

`stock_gripper_body_F_envelope.stl` replaces only the old gripper body AABB.
The official `gripper_base.dae` is fixed at physical J6 with zero origin in
`parallel.urdf.xacro`. Both nested geometry instances were retained: 690 triangles.
Source SHA-256 is `71a616ec936ae9cede3a893a9b5dc0a218f49ce931c7e6ebcbf45c1fba9b6194`.
The new closed convex hull encloses every source vertex plus all corners of a
2 mm axis allowance. Small numerical expansion is included; the serialized mesh
was checked against the original expanded CAD. Its 359 vertices/714 triangles
form a closed mesh with two incident faces per edge and Euler characteristic 2.
The matching JSON records construction, bounds, hashes and offline comparisons.

At the newest run's last J6 angle (~6.214 degrees), the old body box has only
~0.004 mm minimum separating projection from the camera boxes. At 6.8 degrees,
nine boxes overlap. The new body remains separated at the sampled 5.62–15 degree
angles (minimum separating projections ~1.82–2.33 mm). These are comparisons of
this proxy pair only, not physical clearance measurements or full-route validation.
The exact rejected next target was not logged by that revision.

The finger sweep and camera mesh are unchanged. Camera/gripper and camera/link
collision checks remain enabled. The replacement body's SHA-256 is
`bed7a9debe3d7fc1472cc8debd6eeb71a8dd7ece4d53fd466523df44d09e47ba`.
No RoboDK program, tests, hardware connection, live settings or motion was run.

The user confirmed that the camera has **never moved** from its original position.
The previous symmetric +/-70 mm box has been removed from the active setup.

## CAD evidence

The local stock asset `C:\path\to\RoboDK\Denso-Cobotta.robot` was
loaded into isolated, hidden RoboDK instances with no driver or station program.
Its link geometry was exported using the documented `Convert Object` command on
disposable copies. This was CAD inspection, not a calibration or motion test.

Link 4 is the forearm casing. Link 5 contains the camera, bracket, loop cable,
fasteners and wrist casing. Its mesh separates into six connected components:
camera 16,591 triangles, cable 3,974, wrist casing 3,354, bracket 2,234 and two
96-triangle fasteners. The wrist casing was removed from the added camera proxy.

The mesh's DH link-5 coordinates were converted into the published DENSO ROS J5
frame. The constant mapping is rotation `[[-1,0,0],[0,0,1],[0,1,0]]` and translation
`[0,-44.5,0]` mm. It follows from the exported link frame and DENSO URDF at the
asset's stored joints `[0,0,90,0,0,0]`.

The camera is attached before J6. Calibration now estimates `T_mount_camera` in
the J5 frame, and picking computes the camera pose from the first five joints.
The separate grasp TCP remains in the physical J6 flange frame.

## Conservative enclosure

DENSO outline drawing 4004, page 3, gives two factory camera positions. Their
longitudinal extents are -28.7 to +61.65 mm and -47.2 to +43.15 mm: an 18.5 mm
shift. The enclosure sweeps the extracted assembly over that interval and
adds 2 mm in each local axis.

The original 335-box enclosure grouped triangles by their centroid in 8 mm
cells. The user-run session `20261028T214141271643Z_calibrate` exposed a false
camera/gripper-body overlap. Two of those groups mixed triangle corners at
different heights: their source triangles spanned about 29.6 mm in J5 Z. Their
boxes filled empty space near the gripper mounting end. The existing gripper
body box also fills the corners around a rounded body; it is preserved here.

Revision `2026-10-28-refined-camera-envelope` subdivides each retained source
triangle until its span is <=4 mm along every axis, then groups the pieces by
4 mm centroid cells. Each original group is refined separately. Every replacement
box lies within its original inflated/swept parent before float serialization;
all original source surfaces and intermediate mounting-profile positions remain
enclosed. The 2 mm allowance and 18.5 mm sweep are unchanged. Float32 bounds are
rounded outward. No camera/gripper collision pair is excluded.

The current asset contains 3,387 boxes / 40,644 triangles, enclosing the same
22,991 retained source triangles through 81,686 subtriangles. SHA-256:
`9daf6e4b13f3ce0ee7afc1d5ad41ca06ea1fb8390593dec2502ebc0fbceb229d`.
Offline separating-axis analysis at logged J6=-0.37860018699889186 degrees
separates the refined camera enclosure from the unchanged inflated gripper-body
box. The smallest per-box separating projection gap is approximately 1.889 mm.
This is a geometric analysis of this pair/configuration, not a RoboDK planning
run or validation of other collisions, calibration, picking or runtime performance.

`collision_assets/factory_camera_J5_envelope.json` records source hashes, counts,
frame, allowances and limitations. The entry point checks the STL hash. It checks
the enclosure against links 0-4 and 6, gripper proxies, held cube, table and other
station objects. Only the camera's intended link-5 attachment is excluded.
The stock model's own collision checks also remain active.

Sources:
- [RoboDK stock COBOTTA asset](https://robodk.com/robot/Denso/Cobotta)
- [DENSO outline 4004, manufacturer drawing mirror](https://www.dehomespace.top/RobotManual/RC8/Cobotta_jp/img/004004/cobotta_size.pdf)
- [Official DENSO robot description](https://github.com/DENSORobot/denso_cobotta_ros)
- [Canon catalogue, PDF page 5: N10-W02 body 91 x 45 x 38 mm](https://www.canon.com.cn/product/surveillance/down/download/download/Canon_surveillance_201912.pdf)

The retained CAD was processed directly; no CAD re-export or live application
was used for this refinement. The build and inspection sources are in the new
chat workspace's `work/`; the previous asset/source/docs are preserved under
`backups/camera_gripper_refinement_20261028T214908Z/`.

This covers the factory assembly and CAD cable route. Displaced loose cables,
extra fixtures and people are not covered. The program has not been run or tested;
no hardware was connected or commanded during this correction.

## Table mounting and paper area

The user estimates a 10 mm mounting rise. The standard DENSO upper/lower baseplate
assembly drawing gives 35 mm; the photos resemble that assembly but do not resolve
the height. Initial table Z remains -22.5 +/-17.5 mm, covering -40 to -5 mm. The
precalibration collision box uses the highest possible surface, -5 mm. Accepted
calibration must establish the actual printed/table plane within that interval.

[DENSO baseplate assembly drawing](https://www.dehomespace.top/RobotManual/RC8/Cobotta_jp/img/004004/BasePlate_Size.pdf)

The user's 70 mm left, 140 mm right, 40 mm behind and 240 mm toward the user describe
the paper workspace from saved VIEW. They are separate from camera dimensions.
The saved VIEW and 100% arm command settings have been retained.
