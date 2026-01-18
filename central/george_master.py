### import libraries ###
import cv2
import socket
import time
import apriltag
import math
import numpy as np

from CBF import drive
from OnStage_Rcoords import updatePositions
from WifiComms import write, read

### setup camera ###
# logitech
cap = cv2.VideoCapture(0)  # 0 = default USB webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# picam
#from picamera2 import Picamera2

### AprilTag lib ###
module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag

states = ["mining", "deposit", "withdraw", "plant", "none"]

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

class robot:
    def __init__(self, IP, port, x, y, tag):
        self.IP = IP;
        self.port = port
        self.coords = Point(x, y)
        self.water_level = 1
        self.state = "none"
        self.tag = tag;
    def setTarget(x, y):
        self.target = Point(x, y)
    def depleteWater():
        self.water_level -= 1
    def restoreWater():
        self.water_level += 1
    def atTarget():
        return (math.sqrt((self.x-self.targetX)**2 + (self.y-self.targetY)**2) < 0.5)
    
    def setState(int s): #set state manually
        self.state = states[s]
    def action(): #change state after completing current task
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

robots = [robot("192.168.32.172", 5000, 0, 0, 2), robot("192.168.32.172", 5000, 0, 0, 3)] 
anchors = [Point(0, 0), Point(0, 0)]  #AT tag 0 and 1 respectively
obstacles = [obstacle(0, 0, 0), obstacle(0, 0, 0), obstacle(0, 0, 0)]
plants = [plant("<INSERT IP HERE>", 0, 0, 0, 0)]

field_width = 1  #scales x dimension of relative coordinates
field_length = 1  #scales y dimension of relative coordinates
robotcount = 2  #predefined number of robots (prevent detection of extraneous tags)

sockets = []

### functions ###
def connect():
    for robot in robots:
        sockets.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
        sockets[-1].connect((robot.IP, robot.port))

def updatePositions():
    # setup AprilTag detector
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)

    # get image (logitech cam)
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        return
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # get image (picam)
    #yuv420 = picam2.capture_array("lores")
    #rgb = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2RGB)
    #gray = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2GRAY)
    
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
    break

### main ###
if __name__ == "__main__":
    while True:
        for idx in range(len(robots)):
            robot = robots[idx]
            robot.action()
            velocity = drive(robot, np.array(robot.targetX, robot.targetY), obstacles, alpha=1.0, speed=0.10, obs_clearance=0.01, detect_clearance=0.10)
            write(sockets[idx], "vx: " + velocity[0] + ", vy: " + velocity[1])
        updatePositions()

cv2.destroyAllWindows()
picam2.stop()
