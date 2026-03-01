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
import math

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
    
### functions ###
def apply_imgfx(input_img, brightness = 0, contrast = 0):
    if brightness != 0:
        if brightness > 0:
            shadow = brightness
            highlight = 255
        else:
            shadow = 0
            highlight = 255 + brightness
        alpha_b = (highlight - shadow)/255
        gamma_b = shadow
        
        buf = cv2.addWeighted(input_img, alpha_b, input_img, 0, gamma_b)
    else:
        buf = input_img.copy()
    
    if contrast != 0:
        f = 131*(contrast + 127)/(127*(131-contrast))
        alpha_c = f
        gamma_c = 127*(1-f)
        
        buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)

    return buf

def apriltag_rot(result):
    (ptA, ptB, ptC, ptD) = r.corners
    ptB = Point(int(ptB[0]), int(ptB[1]))
    ptC = Point(int(ptC[0]), int(ptC[1]))
    ptD = Point(int(ptD[0]), int(ptD[1]))
    ptA = Point(int(ptA[0]), int(ptA[1]))
    
    # determine 2 corners with greatest y values
    
    deg = math.atan(abs(ptA.y - ptB.y)/abs(ptA.x-ptB.x))
    
    # if A.y > B.y, set deg = -deg
    # else set deg = +deg
    
    return deg
    
### setup camera ###
camera_type = "ausdom"	
camera_port = 0
camera_width = 640; camera_height = 480  #seems that setting camera dimensions doesnt affect resolution, only affects size of camera window
camera_fps = 30;  #capped out at 30 fps for ausdom camera
camera_brightness = 0; camera_contrast = 0

cam = cv2.VideoCapture(camera_port)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
cam.set(cv2.CAP_PROP_FPS, camera_fps)
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
if not cam.isOpened():
    print("Failed to open camera")
    exit()

### main ###
while True:
    ### get image as RGB and GRAY ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        break
    frame = apply_imgfx(frame, camera_brightness, camera_contrast) 
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
cam.release()
