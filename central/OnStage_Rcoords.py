### INFO ###
# install apriltag library by creating a virtual environment and pip install
# make sure to change "module_dir" to match path where library is located

# change "camera_type" to match camera being used e.x. "picam", "ausdom", "logitech"...
# change "camera_port" to match camera usb port
#    can be checked using linux commands
#    sudo apt install v4l-utils
#    v4l2-ctl --list-devices

### import libraries ###
import cv2
import apriltag
import numpy as np

module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag

import math
from OnStage_Master import Point

### setup camera ###
camera_type = "ausdom"
camera_port = 8
camera_width = 640; camera_height = 480
camera_fps = 30;
camera_brightness = 0; camera_contrast = 0

cam = cv2.VideoCapture(camera_port)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
cam.set(cv2.CAP_PROP_FPS, camera_fps)
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
if not cam.isOpened():
    print("Failed to open camera")
    exit()

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
item
    return buf

def displayTags(img, results):
    ### draw bounding boxes and info of ATs ###
    for r in results:
        # get points
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw lines
        cv2.line(img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(img, ptD, ptA, (0, 255, 0), 2)
        # draw center
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
        # draw tag family
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(img, (str(r.tag_id) + " " + tagFamily), (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
   
    ### display edited image ###
    cv2.imshow("Camera", rgb)
    return

def apriltag_rot(result):
    (ptA, ptB, ptC, ptD) = r.corners
    A = Point(int(ptA[0]), int(ptA[1])) # top left corner when AT has 0 rotation
    B = Point(int(ptB[0]), int(ptB[1])) # top right corner
    center = Point(int(r.center[0]), int(r.center[1]))
    
    side_length = A.distance_to(B)
    A2 = Point(center.x - side_length/2, center.y - side_length/2)
    
    base = A.distance_to(A2)
    leg = A.distance_to(center)
    val = (base/2)/leg
    if (val > 1):
        val = 1
    if (val < -1):
        val = -1
        
    rad = 2 * math.asin(val)
    deg = rad * 180 / math.pi
    if (A.y < B.y):
        deg = deg
    elif (A.y > B.y):
        deg = -deg
    else:
        deg = deg
    return deg
    
def convertPos(original_coords, anc_topleft, anc_topright, anc_bottomleft, field_size)
    ###
    new_coords = Point(field_size*(abs(anc_topleft.x - original_coords.x) / abs(anc_topleft.x - anc_topright.x)), field_size*(abs(anc_topleft.y - original_coords.y) / abs(anc_topleft.y - anc_bottomleft.y)))
    return new_coords
    
def updRobotPos(robots):
    ### get image as RGB and GRAY ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        break
    frame = apply_imgfx(frame, camera_brightness, camera_contrast) 
    rgb = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
    options = apriltag.DetectorOptions(families="tag36h11") #setup AT
    detector = apriltag.Detector(options) #setup AT
    results = detector.detect(gray)

    ### update robot x and y values ###
    for r in results:
        for robot in robots:
            if (robot.tag == r.tag_id):
                robot.coords.x = r.center[0]
                robot.coords.y = r.center[1]
                robot.rotation = apriltag_rot(r)
                break
            
    displayTags(rgb, results) #
    return

def initPlantPos(plants):
    ret, frame = cam.read()
    if not ret:
        break
    frame = apply_imgfx(frame, camera_brightness, camera_contrast) 
    rgb = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
    options = apriltag.DetectorOptions(families="tag36h11") #setup AT
    detector = apriltag.Detector(options) #setup AT
    results = detector.detect(gray)

    ### update robot x and y values ###
    for r in results:
        for p in plants:
            if (p.tag == r.tag_id):
                p.coords.x = r.center[0]
                p.coords.y = r.center[1]
                break
            
    displayTags(rgb, results) #
    return
