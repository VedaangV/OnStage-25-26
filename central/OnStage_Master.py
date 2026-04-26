### import libraries ###
import numpy as np
import math
import threading
import socket
import time
import cv2
from munkres import Munkres, print_matrix

### import subfiles ###
from OnStage_Rcoords import updTagPos, updObsPos, initAnchors
from OnStage_WifiComms import wifi_connect, wifi_write, wifi_read, wifi_disconnect
#from OnStage_pfield import pfield_path 
from OnStage_CBF import CBFController, cbf_follow_path, cbf_stop_robot
from OnStage_Audio import play_sound

MAX_PLANT_GROWTH = 4
ENABLE_WIFI = True

states = ["None", "Waiting", "Ice", "Plant"]

# DIST_THRESHOLD = 2

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
    tag: int
    IP: str
    port: int
    sock = None
    coords: Point = Point(-1, -1)
    target: None
    path: None
    rotation: int = 0
    state: str = "None"
    haswater: bool = False
    
    def __init__(self, IP, port, tag):
        self.IP = IP;
        self.port = port
        self.tag = tag;
    def setTarget(self, target):
        self.target = target
    def changeWater(self):
        self.haswater = not self.haswater
    def atTarget(self):
        return (self.coords.distance_to(self.target) < DIST_THRESHOLD) #!tentative

class anchor:
    tag: int
    coords: Point = Point(-1, -1)
    
    def __init__(self, tag):
        self.tag = tag

class obstacle:
    coords: Point = Point(-1, -1)
    radius: int = 0
    border = []

class plant:
    tag: int
    IP: str
    port: int
    sock = None
    coords: Point = Point(-1, -1)
    level: int = 0
    available: bool = True
    
    def __init__(self, IP, port, tag):
        self.IP = IP
        self.port = port
        self.tag = tag
    def grow(self):
        if (self.level < MAX_PLANT_GROWTH):
            self.level = self.level + 1
#             if ENABLE_WIFI == True:
#                 wifi_write(self.sock, "G")

class ice:
    tag: int
    IP: str
    port: int
    sock: None
    coords: Point = Point(-1, -1)
    level: int
    available: bool = True
    
    def __init__(self, tag, level):
        self.tag = tag
        self.level = level
    def deplete(self):
        if (self.level > 0):
            self.level = self.level - 1

robots = [robot("192.168.32.152", 5000, 0)]   #AT tag 5
anchors = [anchor(1), anchor(2), anchor(3)]  #AT tag 0-2

obstacles = []#[obstacle(), obstacle(), obstacle()] #[]
plants = [plant("192.168.32.209", 80, 4)]#[plant("192.168.32.209", 80, 4)]#[plant(WIFI_IP, 80, 4)]  #AT tag 4
icepatches = [ice(5, 1)]  #AT tag 3

obstacle_Lhsv = [160, 225, 130]
obstacle_Uhsv = [180, 255, 240]

field_width = 80  #ft  #scales x dimension of relative coordinates
field_length = 80  #ft  #scales y dimension of relative coordinates
# baseV = 100 #base velocity of robot, m/s

### CBF controller ###
# gamma        : CBF aggressiveness — raise if robots get too close to obstacles/each other
# k_att        : waypoint attraction strength
# safety_margin: extra clearance in field units (on top of robot + obstacle radii)
cbf = CBFController(gamma=0.5, k_att=1.375, safety_margin=3.0)

### setup camera ###
class VideoStream:
    def __init__(self):
        gst_pipeline = (
            "souphttpsrc location=http://192.168.32.214:8080/video is-live=true ! "
            "multipartdemux ! jpegdec ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
        )
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            print("Failed to open camera")
            exit()

        self.ret, self.frame = self.cap.read()
        self.lock = threading.Lock()
        self.running = True

        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret = ret
                    self.frame = frame

    def read(self):
        with self.lock:
            return self.ret, self.frame

    def stop(self):
        self.running = False
        self.cap.release()
        
cam = VideoStream()
        
# camera_port = 0
# camera_width = 640; camera_height = 480
# camera_fps = 30;
# 
# cam = cv2.VideoCapture(camera_port)
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
# cam.set(cv2.CAP_PROP_FPS, camera_fps)
# cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

# if not cam.isOpened():
#     print("Failed to open camera")
#     exit()
    
def displayElements(img, anchors, robots, obstacles, field_size):
    for robot in robots:
        cv2.circle(img, (int(robot.coords.x / field_size * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - robot.coords.y / field_size * (abs(anchors[0].coords.y - anchors[2].coords.y)))), 7, (0, 100, 255), -1)
        for i in range(len(robot.path)):
            cv2.circle(img, (int(robot.path[i][0] / field_size * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - robot.path[i][1] / field_size * (abs(anchors[0].coords.y - anchors[2].coords.y)))), 3, (0, 255, 255), -1)
        if robot.target is not None:
            cv2.circle(img, (int(robot.target.coords.x / field_size * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - robot.target.coords.y / field_size * (abs(anchors[0].coords.y - anchors[2].coords.y)))), 5, (0, 255, 0), -1)
    # add obstacle display 
    return img

# def followPath(robot):
#     if (robot.coords.distance_to(robot.target.coords) < DIST_THRESHOLD):
#         return True
#     if (len(robot.path) > 0 and robot.coords.distance_to(Point(robot.path[0][0], robot.path[0][1])) < DIST_THRESHOLD):
#         robot.path.pop(0)
#     for i in range(len(robot.path)):
#         if (robot.coords.distance_to(Point(robot.path[0][0], robot.path[0][1])) > robot.coords.distance_to(Point(robot.path[i][0], robot.path[i][1]))):
#             for j in range(i):
#                 robot.path.pop(0)
#             break
#     if not robot.path:
#         robot.path.append([robot.target.coords.x, robot.target.coords.y])
#         
# #     print("Robot coords: " + f"{robot.coords.x:.2f}" + ", " + f"{robot.coords.y:.2f}") #testing#
# #     print("Next point coords: " + f"{robot.path[0][0]:.2f}" + ", " + f"{robot.path[0][1]:.2f}") #testing#
# #     print("Target coords: " + f"{robot.target.coords.x:.2f}" + ", " + f"{robot.target.coords.y:.2f}") #testing#
#     dx = robot.path[0][0] - robot.coords.x
#     dy = robot.path[0][1] - robot.coords.y
#     d = math.sqrt(dx**2 + dy**2)
# #     print("dx: " + f"{dx:.2f}" + ", " + "dy: " + f"{dy:.2f}") #testing#
# #     print("dist: " + f"{robot.coords.distance_to(Point(robot.path[0][0], robot.path[0][1])):.2f}") #testing#
# #     print('\n\n') #testing#
#     V = baseV
#     Vx = V * dx / d
#     Vy = V * dy / d
#     if ENABLE_WIFI == True:
#         wifi_write(robot.sock, f"vx: {Vx:.0f}, vy: {Vy:.0f}\n")
#     return False
# 
# def stopRobot(robot):
#     if ENABLE_WIFI == True:
#         wifi_write(robot.sock, "vx: 0, vy: 0\n")
    
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
            print(f"working {row} {column}")
            want_plant[row].target = freeplant[column] 
            freeplant[column].available = False
            want_plant[row].state = "Plant"
        for r in want_plant:
            if r.state != "Plant":
                r.state = "Waiting"
    return
    
### main ###
if __name__ == "__main__":
    ### position camera ###
    while True:
        ret, frame = cam.read()
        if not ret:
            print("Failed to get camera frame")
            continue
        cv2.imshow("Setup", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    
    ### initialize tags/positions ###
    # anchors
    while (res := initAnchors(cam, anchors))[0] != 1:
        for i in range(len(anchors)):
            print(f"Anchor {i}: {anchors[i].coords.x:.2f} {anchors[i].coords.y:.2f}") #testing#
        print("") #testing#
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # plants, ice, robots
    while(res := updTagPos(cam, plants, anchors, field_width))[0] != 1:
        for plant in plants:
            print(f"Plant AT{plant.tag}: {plant.coords.x:.2f} {plant.coords.y:.2f}") #testing#
        print("")
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    while(res := updTagPos(cam, icepatches, anchors, field_width))[0] != 1:
        for ice in icepatches:
            print(f"Ice AT{ice.tag}: {ice.coords.x:.2f} {ice.coords.y:.2f}")
        print("")
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    while(res := updTagPos(cam, robots, anchors, field_width, True))[0] != 1:
        for robot in robots:
            print(f"Robot AT{robot.tag}: {robot.coords.x:.2f} {robot.coords.y:.2f}") #testing#
        print("")
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    if ENABLE_WIFI == True:
        for robot in robots:
            while (robot.sock == None or robot.sock == -1):
                robot.sock = wifi_connect(robot.IP, robot.port)
#         for plant in plants:
#             while (plant.sock == None or plant.sock == -1):
#                 plant.sock = wifi_connect(plant.IP, plant.port)
    
    # obstacles
    while (res := updObsPos(cam, obstacles, obstacle_Lhsv, obstacle_Uhsv, anchors, field_width))[0] != 1:
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue
    
    input("Press Enter to start")
    
    ### get path of points to target ###
    assignTargets(robots, icepatches, plants)
    for robot in robots:
        if (robot.state != "None" and robot.state != "Waiting"):
            cbf_follow_path(cbf, robot, robots, obstacles)
        else:
            continue
    
    ### main loop ###
    while True:
        err, img = updTagPos(cam, robots, anchors, field_width, True)
        for robot in robots:
            if robot.state == "None" or robot.state == "Waiting":
                cbf_stop_robot(robot)
            elif (cbf_follow_path(cbf, robot, robots, obstacles) == True):
                cbf_stop_robot(robot)
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
                    cbf_follow_path(cbf, robot, robots, obstacles)
            else:
                continue
        
        if (err != -1):
            img = displayElements(img, anchors, robots, obstacles, field_width)
            cv2.imshow("Testing", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cv2.destroyAllWindows()
    cam.release()
