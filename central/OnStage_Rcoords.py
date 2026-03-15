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

module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag

os.environ["OPENCV_LOG_LEVEL"] = "OFF"

### objects ###
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other_point):
        dx = self.x - other_point.x
        dy = self.y - other_point.y
        return math.sqrt(dx**2 + dy**2)

    def __str__(self):
        return f"Point({self.x}, {self.y})"

### functions ###
def apriltag_rot(result):
    (ptA, ptB, ptC, ptD) = result.corners
    A = Point(int(ptA[0]), int(ptA[1])) # top left corner when AT has 0 rotation
    B = Point(int(ptB[0]), int(ptB[1])) # top right corner
    center = Point(int(result.center[0]), int(result.center[1]))
    
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
    if (A.y > B.y):
        deg = -deg
    return deg
    
def convertPos(original_coords, anc_topleft, anc_topright, anc_bottomleft, field_size):
    new_coords = Point(field_size*(abs(anc_topleft.x - original_coords.x) / abs(anc_topleft.x - anc_topright.x)), field_size*(abs(anc_topleft.y - original_coords.y) / abs(anc_topleft.y - anc_bottomleft.y)))
    return new_coords

def displayTags(results, img):
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
    return img
    
def initAnchors(cam, anchors):
    ### get image as RGB and GRAY ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        return False
    rgb = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
    options = apriltag.DetectorOptions(families="tag36h11") #setup AT
    detector = apriltag.Detector(options) #setup AT
    results = detector.detect(gray)

    ### update robot x and y values ###
    tags_detected = 0
    for r in results:
        for i in range(len(anchors)):
            if (anchors[i].tag == r.tag_id):
                tags_detected = tags_detected + 1
                anchors[i].coords = Point(r.center[0], r.center[1])
                break
            
    rgb = displayTags(results, rgb)
    cv2.imshow("Camera", rgb)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return
        
    if (tags_detected == len(anchors)):
        return True
    return False
    
def updTagPos(cam, group, anchors, field_size, get_rotation = False):
    ### get image as RGB and GRAY ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        return
    rgb = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
    options = apriltag.DetectorOptions(families="tag36h11") #setup AT
    detector = apriltag.Detector(options) #setup AT
    results = detector.detect(gray)
    
    ### update robot x and y values ###
    for r in results:
        for i in range(len(group)):
            if (group[i].tag == r.tag_id):
                p = Point(r.center[0], r.center[1])
                p = convertPos(p, anchors[0].coords, anchors[1].coords, anchors[2].coords, field_size)
                group[i].coords = p
                if (get_rotation == True):
                    group[i].rotation = apriltag_rot(r)
                break
            
    rgb = displayTags(results, rgb)
    cv2.imshow("Camera", rgb)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return
    return
