# Setup guide

This is the only guide you need. There is **one config file**; the program
creates it for you on the first run.

## What you need

- Windows with RoboDK and a station containing the COBOTTA.
- RoboDK's embedded Python: `C:\RoboDK\Python-Embedded\python.exe`.
- `numpy` 1.25.x and `opencv-contrib-python` 4.5.5 (with `aruco`).
- A DENSO COBOTTA (b-CAP, TCP 5007) and a Canon N10-W02 camera (1920 x 1080).
- The printed `charuco-7x5-letter.pdf` board, actual size.

No administrator rights are needed. Everything is written under your user
profile.

## 1. Clone the repository

Clone it anywhere you can write, for example:

    C:\Users\<you>\Documents\cobottaPickAndLift

Remember this path. You will paste it into the config file.

## 2. Install the Python packages (no admin)

If RoboDK's Python already has OpenCV 4.5.5 and numpy 1.25, skip this step.
Otherwise install them **inside the repository** with `--target`:

    "C:\RoboDK\Python-Embedded\python.exe" -m pip install --target "C:\Users\<you>\Documents\cobottaPickAndLift\vendor" "numpy==1.25.2" "opencv-contrib-python==4.5.5.64"

The program puts `vendor\` first on the Python path, so the packages are used
without touching `C:\RoboDK`. This needs internet access and writes only
inside the repository.

## 3. First run: create the config file

1. In RoboDK: **File > Open** → select `cobotta_calibrate_pick.py` so it becomes
   a Python item. Set the interpreter to
   `C:\RoboDK\Python-Embedded\python.exe`.
2. Run the item once. It creates

       C:\Users\<you>\AppData\Roaming\cobottaPickAndLift\config.json

   with placeholders and stops, printing that path.

## 4. Edit the config file

Open the file printed by the program and fill in these fields:

| Field | Set it to |
| --- | --- |
| `project_directory` | The folder you cloned, e.g. `C:\\Users\\<you>\\Documents\\cobottaPickAndLift` |
| `robot_ip` | The COBOTTA controller IP used by RoboDK / b-CAP |
| `camera_ip` | The camera IP |
| `camera_username` | The camera HTTP user name |
| `camera_password` | The camera HTTP password |

Leave these unless you changed the cell:

- `approach_clearance_mm` (40), `lift_mm` (30)
- `speed_percent` — must stay **100**
- `observation` — the program fills it when you run `set-view`
- `vision` — colour/ROI defaults; edit only if your cube detection needs it

The config file is **per user and outside the repository**, so your IPs and
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
Normal on the first run. Edit the file and run again.

**"Fields to set: ..."**
One or more placeholders are still in the config. Fill them in.

**"The project_directory ... does not contain the whole project"**
`project_directory` does not point at the clone, or the clone is incomplete.
It must contain `cobotta_calibrate_pick.py`, `automatic_setup.json`, the PDFs
and `collision_assets\`.

**"This implementation uses OpenCV 4.5.5 ..."**
Install the pinned packages as in step 2 and run again.

**"Set camera_password ..."**
The camera password field is still the placeholder.

**The program still runs from `%TEMP%`**
That is normal: RoboDK copies the script. The config file's
`project_directory` tells it where the project is, so no environment variable
is needed.
