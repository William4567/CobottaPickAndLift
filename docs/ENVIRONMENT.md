# Environment

- RoboDK embedded Python: `C:\RoboDK\Python-Embedded\python.exe` (per the
  operating guide). Do not point RoboDK at the system Python.
- `opencv-contrib-python` **4.5.5** with `cv2.aruco` is required. The program
  rejects other versions because the ChArUco corner conventions differ.
- `numpy` 1.25.x and RoboDK's own `robodk` package are used.
- Camera: Canon N10-W02 over HTTP (`image.cgi`), 1920 x 1080 JPEG.
- Robot: DENSO COBOTTA, b-CAP over TCP port 5007.

## Installing the packages without administrator rights

`C:\RoboDK` may not be writable for a normal user. Install the two packages
into the repository instead, with `--target`:

    "C:\RoboDK\Python-Embedded\python.exe" -m pip install --target "<clone>\vendor" "numpy==1.25.2" "opencv-contrib-python==4.5.5.64"

The entry point puts `<clone>\vendor` first on `sys.path`, so these packages
are used and nothing under `C:\RoboDK` is modified. This needs internet access
and writes only inside the clone. Skip it if RoboDK's Python already provides
OpenCV 4.5.5 and numpy 1.25.

Do not install other OpenCV versions to satisfy this list; the version check
exists because the ChArUco corner conventions differ.
