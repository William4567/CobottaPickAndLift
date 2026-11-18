# Setup guide

This is the only guide you need. There is **one config file**, in the
repository folder; the program creates it for you on the first run.

## What you need

- Windows with RoboDK and a station containing the COBOTTA.
- RoboDK's embedded Python: `C:\RoboDK\Python-Embedded\python.exe`.
- `numpy` 1.25.x and `opencv-contrib-python` 4.5.5 (with `aruco`).
- A DENSO COBOTTA (b-CAP, TCP 5007) and a Canon N10-W02 camera (1920 x 1080).
- The printed `charuco-7x5-letter.pdf` board, actual size.

No administrator rights are needed.

## 1. Clone the repository

Clone it under your user folder, for example:

    C:\Users\<you>\Documents\cobottaPickAndLift

The program searches for the folder automatically. If you keep it somewhere
unusual, see the troubleshooting section.

## 2. Install the Python packages (no admin)

If RoboDK's Python already has OpenCV 4.5.5 and numpy 1.25, skip this step.
Otherwise install them **inside the repository** with `--target`:

    "C:\RoboDK\Python-Embedded\python.exe" -m pip install --target "C:\Users\<you>\Documents\cobottaPickAndLift\vendor" "numpy==1.25.2" "opencv-contrib-python==4.5.5.64"

The program puts `vendor\` first on the Python path, so nothing under
`C:\RoboDK` is modified. This needs internet access and writes only inside the
repository.

## 3. First run: create the config file

1. In RoboDK: **File > Open** → select `cobotta_calibrate_pick.py` so it becomes
   a Python item. Set the interpreter to
   `C:\RoboDK\Python-Embedded\python.exe`.
2. Run the item once. It creates `config.json` **in the repository folder**,
   next to the script:

       C:\Users\<you>\Documents\cobottaPickAndLift\config.json

   and stops, printing that path.

## 4. Edit the config file

Open the file and fill in these fields:

| Field | Set it to |
| --- | --- |
| `robot_ip` | The COBOTTA controller IP |
| `camera_ip` | The camera IP |
| `camera_username` | The camera HTTP user name |
| `camera_password` | The camera HTTP password |

Leave these unless you changed the cell:

- `approach_clearance_mm` (40), `lift_mm` (30)
- `speed_percent` — must stay **100**
- `observation` — the program fills it when you run `set-view`
- `vision` — colour/ROI defaults; edit only if cube detection needs it

`config.json` is in the repository but is **git-ignored**, so your IPs and
password are never committed. Do not share it.

## 5. Run

Run the Python item again and choose a mode from the dropdown:

1. `status` — reads the controller, no motion. Check the robot and camera.
2. `set-view` — saves the current robot pose as VIEW (no motion).
3. `calibrate` — collects a fresh calibration (board flat and visible).
4. `preview` — locates the cube and plans, without motion.
5. `pick` — physically picks and lifts the cube.

The first calibration needs a VIEW: run `set-view` first. `calibrate` stops
with a clear message if no VIEW is saved.

## Troubleshooting

**"Created a configuration file with placeholders"**
Normal on the first run. Edit the `config.json` it prints and run again.

**"Fields to set: ..."**
One or more placeholders are still in `config.json`. Fill them in.

**"COBOTTA project folder not found"**
The program could not find the cloned folder (RoboDK runs a temporary copy of
the script). Clone it under Documents/Desktop/Downloads, or set the path once
near the top of `cobotta_calibrate_pick.py`:

    PROJECT_DIRECTORY = r"C:\Users\<you>\Documents\cobottaPickAndLift"

**"Several copies of this project were found"**
More than one clone exists. Set `PROJECT_DIRECTORY` to the one you want.

**"This implementation uses OpenCV 4.5.5 ..."**
Install the pinned packages as in step 2 and run again.

**"Set camera_password ..."**
The camera password field is still the placeholder.
