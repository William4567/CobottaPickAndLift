# Environment

- RoboDK embedded Python: `C:\RoboDK\Python-Embedded\python.exe` (per the
  operating guide). Do not point RoboDK at the system Python.
- `opencv-contrib-python` **4.5.5** with `cv2.aruco` is required. The program
  rejects other versions because the ChArUco corner conventions differ.
- `numpy` and RoboDK's own `robodk` package are used.
- Camera: Canon N10-W02 over HTTP (`image.cgi`), 1920 x 1080 JPEG.
- Robot: DENSO COBOTTA, b-CAP over TCP port 5007.

Do not `pip install` into the RoboDK embedded environment to satisfy this list
unless you have verified the resulting OpenCV version. Record any change here.
