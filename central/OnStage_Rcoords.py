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
import copy

module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag

options = apriltag.DetectorOptions(families="tag36h11") #setup AT
detector = apriltag.Detector(options)

os.environ["OPENCV_LOG_LEVEL"] = "OFF"

AREA_THRESHOLD = 5

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
    new_coords = Point(field_size*(abs(anc_topleft.x - original_coords.x) / abs(anc_topleft.x - anc_topright.x)), field_size*(abs(anc_bottomleft.y - original_coords.y) / abs(anc_topleft.y - anc_bottomleft.y)))
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
        return -1
    rgb = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
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
    if tags_detected == len(anchors):
        return 1, rgb
    return 0, rgb
    
def updTagPos(cam, group, anchors, field_size, get_rotation = False):
    ### get image as RGB and GRAY ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        return -1, None
    rgb = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
    results = detector.detect(gray)
    
    ### update robot x and y values ###
    tags_detected = 0
    for r in results:
        for i in range(len(group)):
            if (group[i].tag == r.tag_id):
                tags_detected = tags_detected + 1
                p = Point(r.center[0], r.center[1])
                p = convertPos(p, anchors[0].coords, anchors[1].coords, anchors[2].coords, field_size)
                if p.x > field_size:
                    p.x = field_size
                if p.x < 0:
                    p.x = 0
                if p.y > field_size:
                    p.y = field_size
                if p.y < 0:
                    p.y = 0
                group[i].coords = p
                if (get_rotation == True):
                    group[i].rotation = apriltag_rot(r)
                break
            
    rgb = displayTags(results, rgb)
    if tags_detected == len(group):
        return 1, rgb
    return 0, rgb

canny_LOW = 50
canny_HIGH = 100
def updObsPos(cam, obstacles, Lhsv, Uhsv, anchors, field_size, background = False):
    ### get image as RGB ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        return -1
    
    h, w, c = frame.shape
    alpha = 1.3  #contrast
    beta = -60  #brightness
    frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta) 
    
    ### HSV filter ###
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (Lhsv[0], Lhsv[1], Lhsv[2]), (Uhsv[0], Uhsv[1], Uhsv[2]))
    if background == True:
        frame_threshold = cv2.bitwise_not(frame_threshold)
    
    ### get contours ###
    contours, hierarchy = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    ctrs = np.zeros((h, w, 1), dtype=np.uint8)
    ctrs = np.zeros((h, w, 1), dtype=np.uint8)
    cv2.drawContours(ctrs, contours, -1, 255)
    
    ### filter based on area ###
    temp = []
    for idx in range(len(contours)):
        if cv2.contourArea(contours[idx]) >= AREA_THRESHOLD:
            temp.append(contours[idx])
    contours = temp
    
    obstacle_count = 0
    if (len(contours) == 0):
        return 0, ctrs
    for idx, cnt in enumerate(contours):
        if len(obstacles) == obstacle_count:
            obstacles.append(copy.deepcopy(obstacles[0]))
            obstacles[-1].coords.x = -1
            obstacles[-1].coords.y = -1
            obstacles[-1].border = []
            obstacles[-1].radius = 0
            
        M = cv2.moments(cnt)
        
        ### .coords = centoid ###
        if M["m00"] != 0:
            centroid = Point(int(M['m10']/M['m00']), int(M['m01']/M['m00']))
        else:
            centroid = Point(0, 0)
            
        centroid = convertPos(centroid, anchors[0].coords, anchors[1].coords, anchors[2].coords, field_size)
        obstacles[obstacle_count].coords = centroid
        
        ### .radius = average of max and min dist from centoid along contour ###
        points = cnt.reshape(-1, 2)
        minDist = field_size
        maxDist = 0
        for i in range(len(points)):
            point = Point(points[i][0], points[i][1])
            point = convertPos(point, anchors[0].coords, anchors[1].coords, anchors[2].coords, field_size)
            if (point.distance_to(centroid) < minDist):
                minDist = point.distance_to(centroid)
            if (point.distance_to(centroid) > maxDist):
                maxDist = point.distance_to(centroid)
            ### .border = points along edge of contour ###
            obstacles[obstacle_count].border.append([point.x, point.y]) ### 
        if (minDist == 80 or maxDist == 0):
            return -1
        obstacles[obstacle_count].radius = (minDist + maxDist) / 2
        
        print(str(obstacle_count) + ": " + str(obstacles[obstacle_count].radius)) #testing#
        obstacle_count = obstacle_count + 1
    
    if obstacle_count > 0:
        return 1, ctrs
    return 0, ctrs
