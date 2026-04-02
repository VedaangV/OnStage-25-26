### import libraries ###
import numpy as np
import math
import socket
import time
import cv2
from munkres import Munkres, print_matrix

### import subfiles ###
from OnStage_Rcoords import updTagPos, updObsPos, initAnchors
from OnStage_WifiComms import wifi_connect, wifi_write, wifi_read, wifi_disconnect
from OnStage_pfield import pfield_path

MAX_PLANT_GROWTH = 4

states = ["None", "Waiting", "Ice", "Plant"]

WIFI_IP = "192.168.32.209" #StormingKids

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
    def __init__(self, IP, port, tag):
        self.IP = IP;
        self.port = port
        self.sock = None
        self.coords = Point(-1, -1)
        self.path = None
        self.target = None
        self.rotation = 0
        self.haswater = False
        self.state = "None"
        self.tag = tag;
    def setTarget(self, target):
        self.target = target
    def changeWater(self):
        self.haswater = not self.haswater
    def atTarget(self):
        return (self.coords.distance_to(self.target) < 1) #!tentative

class anchor:
    def __init__(self, tag):
        self.coords = Point(-1, -1)
        self.tag = tag

class obstacle:
    def __init__(self):
        self.coords = Point(-1, -1)
        self.radius = 0
class plant:
    def __init__(self, IP, port, tag):
        self.IP = IP
        self.port = port
        self.sock = None
        self.coords = Point(-1, -1)
        self.tag = tag
        self.available = True
        self.level = 0
    def grow(self):
        if (self.level < MAX_PLANT_GROWTH):
            self.level = self.level + 1
            wifi_write(self.sock, "G")
        
class ice:
    def __init__(self, tag, level):
        self.coords = Point(-1, -1)
        self.tag = tag
        self.available = True
        self.level = level
    def deplete(self):
        if (self.level > 0):
            self.level = self.level - 1

#     def withdraw(self):
#         if (self.level <= 0):
#             return False
#         self.level = self.level - 1
#         return True

robots = [robot(WIFI_IP, 5000, 5)]   #AT tag 5
anchors = [anchor(0), anchor(1), anchor(2)]  #AT tag 0-2

obstacles = [obstacle()] 
plants = [plant(WIFI_IP, 80, 4)]  #AT tag 4
icepatches = [ice(3, 4)]  #AT tag 3

obstacle_Lhsv = [0, 30, 200]
obstacle_Uhsv = [20, 90, 255]

field_width = 80  #scales x dimension of relative coordinates
field_length = 80  #scales y dimension of relative coordinates
baseV = 0.33 #base velocity of robot, m/s

### setup camera ###
camera_port = 0
camera_width = 640; camera_height = 480
camera_fps = 30;

cam = cv2.VideoCapture(camera_port)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
cam.set(cv2.CAP_PROP_FPS, camera_fps)
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
if not cam.isOpened():
    print("Failed to open camera")
    exit()
    
def displayElements(cam, anchors, robots):
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        return -1
    rgb = frame
    
    for anchor in anchors:
        cv2.circle(img, (anchor.coords.x, anchor.coords.y), 5, (0, 0, 255), -1)

    for robot in robots:
        cv2.circle(img, (robot.coords.x, robot.coords.y), 7, (0, 100, 255), -1)
        for i in len(robot.path):
            cv2.circle(img, (robot.path[i][0], robot.path[i][1]), 3, (0, 255, 255), -1)
        cv2.circle(img, (robot.target.coords.x, robot.target.coords.y), 5, (0, 255, 0), -1)
        
    cv2.imshow("Camera", rgb)
    return

def followPath(robot):
    if (robot.coords.distance_to(robot.target.coords) < 1):
        return True
    if (len(robot.path) > 0 and robot.coords.distance_to(Point(robot.path[0][0], robot.path[0][1])) < 1):
        robot.path.pop(0)
    if not robot.path:
        robot.path.append([robot.target.coords.x, robot.target.coords.y])
        
    print("Robot coords: " + f"{robots[0].coords.x:.2f}" + ", " + f"{robots[0].coords.y:.2f}") #testing#
    print("Next point coords: " + f"{robot.path[0][0]:.2f}" + ", " + f"{robot.path[0][1]:.2f}") #testing#
    dx = robot.path[0][0] - robot.coords.x
    dy = robot.path[0][1] - robot.coords.y
    d = math.sqrt(dx**2 + dy**2)
    print("dx: " + f"{dx:.2f}" + ", " + "dy: " + f"{dy:.2f}") #testing#
    print("dist: " + f"{robots[0].coords.distance_to(Point(robot.path[0][0], robot.path[0][1])):.2f}") #testing#
    print('\n') #testing#
    V = baseV
    Vx = V * dx / d
    Vy = V * dy / d
    #wifi_write(robots[0].sock, f"{Vx:.2f},{Vy:.2f}")
    return False

def assignTargets(robots, icepatches, plants): # system = plants or ice
    ### assign targets using matrix calculations ###
    # intial calculation of closest robot targets (minimal movement)
    # use Hungarian algorithm (O(n^3)): uses cost matrix to maximize efficiency 
    want_plant = []
    want_ice = []
    freeplant = []
    freeice = []
    for r in robots:
        if (r.state == "None" or r.state == "Waiting"):
            if r.haswater == False:
                want_ice.append(r)
            else:
                want_plant.append(r)
    for i in icepatches:
        if (i.available == True):
            freeice.append(i)
    for p in plants:
        if (p.available == True):
            freeplant.append(p)
    
    # ice #
    if (len(want_ice) > 0):
        matrix = []
        row = 0
        for r in want_ice:
            matrix.append([])
            for i in freeice:
                matrix[row].append(r.coords.distance_to(i.coords))
            row += 1
        
        m = Munkres()
        indexes = m.compute(matrix)
        #print_matrix(matrix, msg='Lowest cost through this matrix:')
        total = 0
        for row, column in indexes:
            want_ice[row].target = freeice[column] 
            freeice[column].available = False
            want_ice[row].state = "Ice"
        for r in want_ice:
            if r.state != "Ice":
                r.state = "Waiting"
    
    # plants #
    if (len(want_plant) > 0):
        matrix = []
        row = 0
        for r in want_plant:
            matrix.append([])
            for p in freeplant:
                matrix[row].append(r.coords.distance_to(p.coords))
            row += 1
        
        m = Munkres()
        indexes = m.compute(matrix)
        #print_matrix(matrix, msg='Lowest cost through this matrix:')
        total = 0
        for row, column in indexes:
            want_plant[row].target = freeplant[column] 
            freeplant[column].available = False
            want_plant[row].state = "Plant"
        for r in want_plant:
            if r.state != "Plant":
                r.state = "Waiting"
    return
    
### main ###
if __name__ == "__main__":
    ### initialize tags/positions ###
    # anchors
    while (initAnchors(cam, anchors) != 1):
        for i in range(len(anchors)):
            print(f"Anchor {i}: {anchors[i].coords.x:.2f} {anchors[i].coords.y:.2f}") #testing#
        print("\n") #testing#
        continue
    
    # plants, ice, robots
    counter = 0
    while (counter != (len(robots)+len(icepatches)+len(plants))):
        counter = 0
        updTagPos(cam, plants, anchors, field_width)
        updTagPos(cam, icepatches, anchors, field_width)
        updTagPos(cam, robots, anchors, field_width, True)
        for robot in robots:
            if robot.coords != Point(-1, -1):
                counter = counter + 1
            print(f"Robot AT{robot.tag}: {robot.coords.x:.2f} {robot.coords.y:.2f}") #testing#
        for ice in icepatches:
            if ice.coords != Point(-1, -1):
                counter = counter + 1
            print(f"Ice AT{ice.tag}: {ice.coords.x:.2f} {ice.coords.y:.2f}") #testing#
        for plant in plants:
            if plant.coords != Point(-1, -1):
                counter = counter + 1
            print(f"Plant AT{plant.tag}: {plant.coords.x:.2f} {plant.coords.y:.2f}") #testing#
        print("\n") #testing#

#     for robot in robots:
#         while (robot.sock == None or robot.sock == -1):
#             robot.sock = wifi_connect(robot.IP, robot.port)
#     for plant in plants:
#         while (plant.sock == None or plant.sock == -1):
#             plant.sock = wifi_connect(plant.IP, plant.port)
    
    # obstacles
    while (updObsPos(cam, obstacles, obstacle_Lhsv, obstacle_Uhsv, anchors, field_width) != 1):
        continue
    
    input("Press Enter to start")
    
    assignTargets(robots, icepatches, plants)
    
    while True:
        ### get path of points to target ###
        for robot in robots:
            if (robot.state != "None" and robot.state != "Waiting"):
                robot.path = pfield_path(robots[0], obstacles, field_width)
                followPath(robot)
        
        for robot in robots:
            if (followPath(robot) == True):
                if (robot.state == "Ice"):
                    robot.target.deplete()
                    if robot.target.level == 0:
                        robot.target.available = False
                    robot.haswater = True
                if (robot.state == "Plant"):
                    robot.target.grow()
                    if robot.target.level == MAX_PLANT_GROWTH:
                        robot.target.available = False
                    robot.haswater = False
                robot.state = "None"
                robot.target = None
                assignTargets(robots, icepatches, plants)
                if (robot.state != "None" and robot.state != "Waiting"):
                    robot.path = pfield_path(robots[0], obstacles, field_width)
    
    cv2.destroyAllWindows()
    cam.release()
