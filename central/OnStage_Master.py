### import libraries ###
import numpy as np
import math
import socket
import time
import cv2
from munkres import Munkres, print_matrix

### import subfiles ###
from OnStage_Rcoords import updTagPos, updObsPos, initAnchors
#from OnStage_WifiComms import wifi_write, wifi_read
from OnStage_pfield import pfield_path

MAX_PLANT_GROWTH = 4

states = ["None", "Waiting", "Ice", "Plant"]

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
        self.path = [[-1, -1]]
        self.target = Point(-1, -1)
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
    
    def setState(self, s): #set state manually
        self.state = s
#     def changeState(self): #change state after completing current task
#         if (self.atTarget == True):
#             if self.state == "Plant" and self.haswater:
#                 if self.target.grow():
#                     self.changeWater()
#                     self.state = "Ice"
#                 else:
#                     idx = plants.index(self.target)
#                     plants.pop(idx)
#                     plant_state.pop(idx)
#                     self.state = "Plant"
#             elif self.state == "Ice" and not self.haswater:
#                 if self.target.withdraw():
#                     self.changeWater()
#                     self.state = "Plant"
#                 else:
#                     idx = ice_patches.index(self.target)
#                     ice_patches.pop(idx)
#                     ice_state.pop(idx)
#                     self.state = "Ice"
#             return True
#         elif self.state == "Waiting for Ice" or self.state == "Waiting for Plant":
#             return True
#         else:
#             return False

class anchor:
    def __init__(self, x, y, tag):
        self.coords = Point(x, y)
        self.tag = tag

class obstacle:
    def __init__(self, x, y, radius):
        self.coords = Point(x, y)
        self.radius = radius
class plant:
    def __init__(self, IP, port, x, y, tag, level):
        self.IP = IP;
        self.port = port
        self.coords = Point(x, y)
        self.tag = tag
        self.available = True
        self.level = level
    def grow(self):
        if (self.level < MAX_PLANT_GROWTH):
            if (wifi_write("G") == -1):
                return False
            else:
                return True
        return False
        
class ice:
    def __init__(self, x, y, tag, level):
        self.coords = Point(x, y)
        self.tag = tag
        self.available = True
        self.level = level

#     def withdraw(self):
#         if (self.level <= 0):
#             return False
#         self.level = self.level - 1
#         return True

robots = [robot("192.168.32.172", 5000, -1, -1, 4)]   #AT tag 4-5
anchors = [anchor(-1, -1, 0), anchor(-1, -1, 1), anchor(-1, -1, 2), anchor(-1, -1, 3)]  #AT tag 0-3

obstacles = [obstacle(40, 40, 3)] #obstacle(-1, -1, 0), obstacle(-1, -1, 0), obstacle(-1, -1, 0)]
plants = [plant("<INSERT IP HERE>", 0, -1, -1, 5, 0)]
icepatches = [ice(-1, -1, 0, 4)]

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

sockets = []

def followPath(robot):
    while (robot.atTarget() == False):
        if not robot.path:
            robot.append([robot.target.x, robot.target.y])
        while (robot.coords.distance_to(Point(robot.path[0][0], robot.path[0][1])) >= 1):
            updTagPos(cam, robots, anchors, field_width, True)
            print("Robot 0 coords: " + f"{robots[0].coords.x:.2f}" + ", " + f"{robots[0].coords.y:.2f}") #testing#
            print("Point coords: " + f"{points[idx][0]:.2f}" + ", " + f"{points[idx][1]:.2f}") #testing#
            dx = robot.path[0][0] - robot.coords.x
            dy = robot.path[0][1] - robot.coords.y
            d = math.sqrt(dx**2 + dy**2)
            print("dx: " + f"{dx:.2f}" + ", " + "dy: " + f"{dy:.2f}") #testing#
            print("dist: " + f"{robots[0].coords.distance_to(Point(points[idx][0], points[idx][1])):.2f}") #testing#
            print('\n') #testing#
            V = baseV
            Vx = V * dx / d
            Vy = V * dy / d
            #wifi_write(f"{Vx:.2f}" + "," + f"{Vy:.2f}")
        robot.path.pop(0)
    return

# def assignTargets(system): # system = plants or ice
#     ### assign targets using matrix calculations ###
#     # intial calculation of closest robot targets (minimal movement)
#     # use Hungarian algorithm (O(n^3)): uses cost matrix to maximize efficiency 
# 
#     # initialize cost matrix
#     matrix = []
#     row = 0
#     for r in robots:
#         matrix.append([])
#         for item in system:
#             matrix[row].append(r.coords.distance_to(item.coords))
#         row += 1
#     
#     m = Munkres()
#     indexes = m.compute(matrix)
#     #print_matrix(matrix, msg='Lowest cost through this matrix:')
#     total = 0
#     for row, column in indexes:
#         robots[row].setTarget(system[column].coords)
#         #value = matrix[row][column]
#         #total += value
#         #print(f'({row}, {column}) -> {value}')
#     #print(f'total cost: {total}')
        
# def setNewTarget(robot):
#     if robot.state == "Ice":
#         for idx, ice in ice_patches:
#             if not ice_state[idx]:
#                 robot.setTarget(ice)
#                 return
#         robot.state = "Waiting for Ice"
#     elif robot.state == "Plant":
#         for idx, plant in plants:
#             if not plant_state[idx]:
#                 robot.setTarget(plant)
#                 return
#         robot.state = "Waiting for Plant"

    # if it doesn't find any available, we may need to just keep waiting

def connect(system): #systems are the arrays with objects with have .IP and .port (robots, plants)
    for item in system:
        sockets.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
        sockets[-1].connect((item.IP, item.port))

### main ###
if __name__ == "__main__":
    ### initialize tags/positions ###
    # anchors
    while (initAnchors(cam, anchors) != 1):
        print("Top left: " + str(anchors[0].coords.x) + " " + str(anchors[0].coords.y))
        print("Top right: " + str(anchors[1].coords.x) + " " + str(anchors[1].coords.y))
        print("Bottom left: " + str(anchors[2].coords.x) + " " + str(anchors[2].coords.y))
        print("Bottom right: " + str(anchors[3].coords.x) + " " + str(anchors[3].coords.y))
        continue
    
    # plants, ice, robots
    while (robots[0].coords.x == -1 or robots[0].coords.y == -1 or plants[0].coords.x == -1 or plants[0].coords.y == -1):
        updTagPos(cam, plants, anchors, field_width)
        updTagPos(cam, robots, anchors, field_width, True)
        print("Robot 0 coords: " + str(robots[0].coords.x) + " " + str(robots[0].coords.y))
        print("Plant 0 coords: " + str(plants[0].coords.x) + " " + str(plants[0].coords.y))
    
    # obstacles
    while (updObsPos(cam, obstacles, obstacle_Lhsv, obstacle_Uhsv, anchors, field_width) == -1):
        continue
    
#     ### assign targets using matrix calculations ###
#     # intial calculation of closest robot targets (minimal movement)
#     # use Hungarian algorithm (O(n^3)): uses cost matrix to maximize efficiency 
# 
#     # initialize cost matrix
#     matrix = []
#     row = 0
#     for r in robots:
#         r.setState("Ice")
#         matrix.append([])
#         for i in icepatches:
#             matrix[row].append(r.coords.distance_to(i.coords))
#         row += 1
#     
#     m = Munkres()
#     indexes = m.compute(matrix)
#     #print_matrix(matrix, msg='Lowest cost through this matrix:')
#     total = 0
#     for row, column in indexes:
#         robots[row].setTarget(ice_patches[column].coords.x, ice_patches[column].coords.y)
#         #value = matrix[row][column]
#         #total += value
#         #print(f'({row}, {column}) -> {value}')
#     #print(f'total cost: {total}')

    if (plants[0].coords.x != -1 and plants[0].coords.y != -1):
        robots[0].setTarget(plants[0].coords)
    
    while True:
#             if robots[i].changeState(): # if we have arrived at target, we want to set a new target
#                 setNewTarget(robots[i])
            #velocity = get_velocity(robots[i], obstacles, alpha=1.0, speed=0.10, obs_clearance=0.01, detect_clearance=0.10)
            #print("vx: " + str(velocity[0]) + ", vy: " + str(velocity[1]))
        
        ### get path of points to target ###
        points = pfield_path(robots[0], obstacles, field_width)
        print(points)
        
        print("\n***traversing path***") #testing#
        ### send velocity from current position to next point ###
        for idx in range(len(points)):
            while (robots[0].coords.distance_to(Point(points[idx][0], points[idx][1])) >= 1):
                updTagPos(cam, robots, anchors, field_width, True)
                print("Robot 0 coords: " + f"{robots[0].coords.x:.2f}" + ", " + f"{robots[0].coords.y:.2f}") #testing#
                print("Point coords: " + f"{points[idx][0]:.2f}" + ", " + f"{points[idx][1]:.2f}") #testing#
                dx = points[idx][0] - robots[0].coords.x
                dy = points[idx][1] - robots[0].coords.y
                d = math.sqrt(dx**2 + dy**2)
                print("dx: " + f"{dx:.2f}" + ", " + "dy: " + f"{dy:.2f}") #testing#
                print("dist: " + f"{robots[0].coords.distance_to(Point(points[idx][0], points[idx][1])):.2f}") #testing#
                print('\n') #testing#
                V = baseV
                Vx = V * dx / d
                Vy = V * dy / d
                #write(sockets[i], f"{Vx:.2f}" + "," + f"{Vy:.2f}")
        
        print("\n***approaching target***") #testing#
        ### close distance to target ###
        while (robots[0].atTarget == False):
            updTagPos(cam, robots, anchors, field_width, True)
            print("Robot 0 coords: " + f"{robots[0].coords.x:.2f}" + ", " + f"{robots[0].coords.y:.2f}") #testing#
            print("Target coords: " + f"{robots[0].target.x:.2f}" + ", " + f"{robots[0].target.y:.2f}") #testing#
            dx = robots[0].target.x - robots[0].coords.x
            dy = robots[0].target.y - robots[0].coords.y
            d = math.sqrt(dx**2 + dy**2)
            print("dx: " + f"{dx:.2f}" + ", " + "dy: " + f"{dy:.2f}") #testing# 
            print('\n') #testing#
            V = baseV
            Vx = V * dx / d
            Vy = V * dy / d
            #write(sockets[i], f"{Vx:.2f}" + "," + f"{Vy:.2f}")
        
        #write(sockets[i], "0,0")
        break #switch target here
    
    cv2.destroyAllWindows()
    cam.release()
