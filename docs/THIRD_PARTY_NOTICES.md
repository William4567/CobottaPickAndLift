# Third-party notices

This project is MIT-licensed (see [LICENSE](../LICENSE)) and includes or derives from the
following third-party material.

## DENSO b-CAP Python sample code

`bcapclient.py`, `variant.py` and `orinexception.py` are the DENSO WAVE
b-CAP Python sample implementation:

    Copyright (c) 2017 DENSO WAVE INCORPORATED
    MIT License

The full MIT permission notice is retained at the top of each of those files.

## Gripper collision envelope

`collision_assets/stock_gripper_body_F_envelope.stl` is a convex enclosure of
the stock DENSO gripper CAD from the `denso_cobotta_ros` project
(`cobotta_description/gripper_base.dae`), which is MIT-licensed:

    https://github.com/DENSORobot/denso_cobotta_ros

## Camera collision envelope

`collision_assets/factory_camera_J5_envelope.stl` is a refined enclosure derived
from the Denso Cobotta robot model distributed with RoboDK and the public RC8
Cobotta dimension drawing:

    https://robodk.com/robot/Denso/Cobotta
    https://www.dehomespace.top/RobotManual/RC8/Cobotta_jp/img/004004/cobotta_size.pdf

It contains the factory camera, bracket, fasteners and a CAD cable loop, with a
2 mm allowance and both documented factory mounting profiles. RoboDK and DENSO
terms may apply to redistribution of this derived mesh; review them before
publishing or redistributing it.

## Printed ChArUco board

`charuco-7x5.pdf` and `charuco-7x5-letter.pdf` are
generated from OpenCV's ChArUco board implementation (OpenCV 4.5.5, Apache-2.0):

    https://opencv.org/license/

## RoboDK

The program uses a locally installed RoboDK and its `robodk` Python package.
RoboDK is not included in this repository and remains subject to its own
license.
