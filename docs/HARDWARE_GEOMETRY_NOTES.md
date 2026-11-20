# Hardware Geometry Notes

## Gripper body envelope

`collision_assets/stock_gripper_body_F_envelope.stl` encloses the official stock
`gripper_base.dae` from `denso_cobotta_ros` (690 triangles, SHA-256
`71a616ec936ae9cede3a893a9b5dc0a218f49ce931c7e6ebcbf45c1fba9b6194`) mounted at
physical J6 with zero origin.

- Closed convex hull (359 vertices, 714 triangles, Euler characteristic 2) covering every CAD vertex plus `[-2, +2]^3 mm` corners and 100 ppm outward scaling.
- SHA-256: `bed7a9debe3d7fc1472cc8debd6eeb71a8dd7ece4d53fd466523df44d09e47ba`.
- The finger opening sweep remains a separate axis-aligned box proxy (`[-16, -29, 36]` to `[16, 29, 60]` mm in flange coordinates, inflated by `2 mm`).
- See `collision_assets/stock_gripper_body_F_envelope.json` for bounds and vertex/triangle metrics.

## Camera J5 envelope

`collision_assets/factory_camera_J5_envelope.stl` encloses the stock Canon
N10-W02 camera, mounting bracket, fasteners, and routed cable extracted from link 5
of RoboDK's `Denso-Cobotta.robot` CAD model (22,991 retained source triangles;
the 3,354-triangle wrist casing was separated out).

- Link-5 DH coordinates are converted into the `DENSO_ROS_J5` frame via constant rotation `[[-1, 0, 0], [0, 0, 1], [0, 1, 0]]` and translation `[0, -44.5, 0]` mm.
- Because DENSO outline drawing 4004 lists two factory camera longitudinal positions (`-28.7..+61.65 mm` and `-47.2..+43.15 mm`, an `18.5 mm` shift), the mesh sweeps the assembly across that `18.5 mm` interval and adds `2 mm` margin per axis.
- Source triangles are subdivided to `<= 4 mm` per axis and grouped in `4 mm` cells (3,387 boxes / 40,644 triangles) to prevent long triangles from filling empty space near the wrist mounting end.
- SHA-256: `9daf6e4b13f3ce0ee7afc1d5ad41ca06ea1fb8390593dec2502ebc0fbceb229d`.
- See `collision_assets/factory_camera_J5_envelope.json` for full provenance and counts.

## Active collision pairs

- The camera is attached to J5 (`camera_mount_fk`), before J6 rotation; calibration estimates `T_mount_camera` in `DENSO_ROS_J5`, while `T_flange_grasp` remains in the physical J6 flange frame.
- Camera/link-5 attachment is excluded.
- The `stock_gripper_body_F_envelope` / `factory_camera_J5_envelope` pair is disabled in `Planner.create_geometry()` because both meshes include conservative 2 mm allowances and the 18.5 mm factory profile sweep while physical wrist clearance across J6 rotation was verified on hardware.
- Camera checks against `finger_swept_envelope`, robot links 0–4 and 6, table, held payload, target cube, and station objects remain enabled.
- Displaced loose cables, unmodelled fixtures, and people are outside the CAD model.

## Table mounting height

- Standard DENSO upper/lower baseplate documentation lists a `35 mm` rise; initial table bounds in `automatic_setup.json` use `table_initial_z_mm = -22.5` with `±17.5 mm` uncertainty (covering `-40 mm` to `-5 mm`).
- Pre-calibration collision checks use the conservative upper bound (`-5 mm`). Accepted calibration establishes the measured table plane (`point_base_mm`, `normal_base`) after subtracting `board_thickness_mm`.

## Reference sources

- [RoboDK stock COBOTTA asset](https://robodk.com/robot/Denso/Cobotta)
- [DENSO outline 4004 drawing](https://www.dehomespace.top/RobotManual/RC8/Cobotta_jp/img/004004/cobotta_size.pdf)
- [DENSO baseplate assembly drawing](https://www.dehomespace.top/RobotManual/RC8/Cobotta_jp/img/004004/BasePlate_Size.pdf)
- [Official DENSO COBOTTA ROS description](https://github.com/DENSORobot/denso_cobotta_ros)
- [Canon N10-W02 specifications (catalogue p. 5: 91 x 45 x 38 mm)](https://www.canon.com.cn/product/surveillance/down/download/download/Canon_surveillance_201912.pdf)
