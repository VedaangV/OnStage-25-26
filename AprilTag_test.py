### INFO ###
# install apriltag library by creating a virtual environment and pip install
# make sure to change "module_dir" to match path where library is located

# change "camera_type" to match camera being used e.x. "picam", "ausdom", "logitech"...
# change "camera_port" to match camera usb port
#    can be checked using linux commands
#    sudo apt install v4l-utils
#    v4l2-ctl --list-devices

### import libraries ###
import sys
import os
import cv2
import numpy as np

from picamera2 import Picamera2

module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag

### objects ###
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __array__(self) -> np.ndarray:
        return np.array([self.x, self.y])
    
### setup camera ###
camera_type = "ausdom"
camera_port = 8;

if camera_type == "picam":
    cam = Picamera2()
    config = cam.create_preview_configuration(lores={"size": (640, 480)})   #(640, 480)
    cam.configure(config)
    cam.start()
else:
    cam = cv2.VideoCapture(camera_port)
    if not cam.isOpened():
        print("Failed to open camera")
        exit()

### main ###
while True:
    ### get image as RGB and GRAY ###
    if camera_type == "picam":
        yuv420 = cam.capture_array("lores")
        rgb = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2RGB)
        gray = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2GRAY)
    else:
        ret, frame = cam.read()
        if not ret:
            print("Failed to get camera frame")
            break
        rgb = frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get results of AT detection ###
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
   
    ### draw bounding boxes and info of ATs ###
    for r in results:
        # get points
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw lines
        cv2.line(rgb, ptA, ptB, (0, 255, 0), 2)
        cv2.line(rgb, ptB, ptC, (0, 255, 0), 2)
        cv2.line(rgb, ptC, ptD, (0, 255, 0), 2)
        cv2.line(rgb, ptD, ptA, (0, 255, 0), 2)
        # draw center
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(rgb, (cX, cY), 5, (0, 0, 255), -1)
        # draw tag family
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(rgb, (str(r.tag_id) + " " + tagFamily), (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    ### display edited image ###
    cv2.imshow("Camera", rgb)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

cv2.destroyAllWindows()
if camera_type == "picam":
    cam.stop()
else:
    cam.release()
