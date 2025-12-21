### import libraries ###
import sys
import os
import cv2
import numpy as np
import copy
import math

from picamera2 import Picamera2 #picam

module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag

states = ["mining", "deposit", "withdraw", "plant", "none"]

### objects/classes ###
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
    
class robot:
    def __init__(self, IP, port, x, y, tag):
        self.IP = IP;
        self.port = port
        self.coords = Point(x, y)
        self.tag = tag;
        self.haswater = False
        self.state = states[5]
    def updatePosition(self, x, y):
        self.coords = Point(x, y)
    def setTarget(x, y):
        self.target = Point(x, y)
        
    def changeWater():
        self.haswater = not self.haswater
    def atTarget():
        return (self.coords.distance_to(self.target) < 0.5)
    def changeState(int s):
        self.state = states[s]
    def action():
        if (self.atTarget == True):
            self.changeWater()
            for i in range(4):
                if (self.state == states[i]):
                    self.state == states[(i + 1) % 4]
                    
class obstacle:
    def __init(self, x, y, radius):
        self.coords = Point(x, y)
        self.radius = radius

class plant:
    def __init__(self, IP, port, x, y, level):
        self.IP = IP;
        self.port = port
        self.coords = Point(x,y)
        self.level = level
        
class ice:
    def __init__(self, x, y, level):
        self.coords = Point(x,y)
        self.level = level

### field elements ###
robots = [robot("192.168.32.172", 5000, 0, 0, 2)] 
anchors = [Point(0, 0), Point(0, 0)]  #AT tag 0 and 1 respectively
obstacles = [obstacle(0, 0, 0), obstacle(0, 0, 0), obstacle(0, 0, 0)]
plants = [plant("<INSERT IP HERE>", 0, 0, 0, 0)]

field_width = 1  #scales x dimension of relative coordinates
field_length = 1  #scales y dimension of relative coordinates
robotcount = 1  #predefined number of robots (prevent detection of extraneous tags)


### functions ###
def updatePositions():
    # setup AprilTag detector
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    
    # get image (logitech cam)
#     ret, frame = cap.read()
#     if not ret:
#         print("Failed to grab frame")
#         return
#     rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # get image (picam)
    yuv420 = picam2.capture_array("lores")
    rgb = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2RGB)
    gray = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2GRAY)
    
    # get AT detection results
    results = detector.detect(gray)
    
    for r in results:
        if r.tag_id >= 2 and r.tag_id <= 2 + robotcount - 1:
            robots[r.tag_id-2].coords = Point(field_width*(abs(anchors[0].x - r.center[0]) / abs(anchors[0].x - anchors[1].x)), field_length*(abs(anchors[0].y - r.center[1]) / abs(anchors[0].y - anchors[1].y)))
            
            ###
            # draw bounding box
            cv2.line(rgb, ptA, ptB, (0, 255, 0), 2)
            cv2.line(rgb, ptB, ptC, (0, 255, 0), 2)
            cv2.line(rgb, ptC, ptD, (0, 255, 0), 2)
            cv2.line(rgb, ptD, ptA, (0, 255, 0), 2)
            # draw center
            cv2.circle(rgb, (cX, cY), 5, (0, 0, 255), -1)
            # draw tag family
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(rgb, (str(r.tag_id) + " " + tagFamily), (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # draw anchors
    cv2.circle(rgb, (anchor[0].x, anchor[0].y), 5, (0, 0, 255), -1)
    cv2.circle(rgb, (anchor[1].x, anchor[1].y), 5, (0, 0, 255), -1)
    
    # display image
    cv2.imshow("Camera", rgb)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
    
    

### robot sockets ###
sockets = []
'''
for robot in robots:
    sockets.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
    sockets[-1].connect((robot.IP, robot.port))
    '''
def write(robot, string):
    sockets[robot].send(string.encode())
    
def read(robot):
    return sockets[robot].recv(1024).decode().strip()

### setup picam ###
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640, 480)})   #(640, 480)
picam2.configure(config)
picam2.start()

## adjust camera, set anchor coords ###
while True:
    yuv420 = picam2.capture_array("lores")
    rgb = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2RGB)
    gray = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2GRAY)
    
    ### get results of AT detection ###
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    
    for r in results:
        if r.tag_id >= 0 and r.tag_id <= 1:
            try:
                print(str(r.center[0]) + ", " + str(r.center[1]))
            except NameError:
                continue
            else:
                anchors[r.tag_id] = Point(r.center[0], r.center[1])
            
    ### draw bounding boxes and info of ATs ###
    for r in results:
        if r.tag_id >= 0 and r.tag_id <= 1:
            # get points
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            displayTag(ptA, ptB, ptC, ptD, cX, cY)
    
    ### display edited image ###
    cv2.imshow("Camera", rgb)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
    
### get robot coords ###
while True:
    ### get image as RGB and GRAY ###
    yuv420 = picam2.capture_array("lores")
    rgb = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2RGB)
    gray = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2GRAY)
    
    ### get results of AT detection ###
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)

    for r in results:
        if r.tag_id >= 2 and r.tag_id <= 3:
            robots[r.tag_id-2].coords = Point((abs(anchors[0].x - r.center[0]) / abs(anchors[0].x - anchors[1].x)), (abs(anchors[0].y - r.center[1]) / abs(anchors[0].y - anchors[1].y)))
            print("Robot " + str(r.tag_id - 2) + " relative coords: " + str(robots[r.tag_id-2].coords.x) + ", " + str(robots[r.tag_id-2].coords.y))
    print("\n")
   
    ### draw bounding boxes and info of ATs ###
    for r in results:
        if r.tag_id >= 2 and r.tag_id <= 3:
            # get points
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            displayTag(ptA, ptB, ptC, ptD, cX, cY)
            cv2.circle(rgb, (anchor[0].x, anchor[0].y), 5, (0, 0, 255), -1)
            cv2.circle(rgb, (anchor[1].x, anchor[1].y), 5, (0, 0, 255), -1)
    
    ### display edited image ###
    cv2.imshow("Camera", rgb)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

cv2.destroyAllWindows()
picam2.stop()
