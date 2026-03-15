### import libraries ###
import numpy as np
import math
import socket

### import subfiles ###
#from CBF import get_velocity
from OnStage_Rcoords import updTagPos, initAnchors
#from OnStage_WifiComms import write, read

# from munkres import Munkres, print_matrix

MAX_PLANT_GROWTH = 4

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
        self.target = Point(0, 0)
        self.rotation = 0
        self.haswater = False
        self.state = "Ice"
        self.tag = tag;
    def setTarget(self, target):
        self.target = target
    def changeWater(self):
        self.haswater = not self.haswater
    def atTarget(self):
        return (math.sqrt((self.coords.x-self.target.x)**2 + (self.coords.y-self.target.y)**2) < 0.5)  #!tentative
    
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
        self.type = "Plant"

    def grow(self):
        if (self.level < MAX_PLANT_GROWTH):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((self.IP, self.port))
            write(sock, 'G')
            return True
        return False
        
class ice:
    def __init__(self, x, y, level):
        self.coords = Point(x, y)
        self.available = True
        self.level = level
        self.type = "Ice"

    def withdraw(self):
        if (self.level <= 0):
            return False
        self.level = self.level - 1
        return True

robots = [robot("192.168.32.172", 5000, 0, 0, 4), robot("192.168.32.172", 5000, 0, 0, 5)]   #AT tag 4-5
anchors = [anchor(0, 0, 0), anchor(0, 0, 1), anchor(0, 0, 2), anchor(0, 0, 3)]  #AT tag 0-3

obstacles = [obstacle(0, 0, 0), obstacle(0, 0, 0), obstacle(0, 0, 0)]
plants = [plant("<INSERT IP HERE>", 0, 0, 0, 8, 0)]
icepatches = [ice(0, 0, 0)]

field_width = 8  #scales x dimension of relative coordinates
field_length = 8 #scales y dimension of relative coordinates
robotcount = 2  #predefined number of robots (prevent detection of extraneous tags)

camera_port = 0

sockets = []

def setNewTarget(robot):
    if robot.state == "Ice":
        for idx, ice in ice_patches:
            if not ice_state[idx]:
                robot.setTarget(ice)
                return
        robot.state = "Waiting for Ice"
    elif robot.state == "Plant":
        for idx, plant in plants:
            if not plant_state[idx]:
                robot.setTarget(plant)
                return
        robot.state = "Waiting for Plant"

    # if it doesn't find any available, we may need to just keep waiting
        

def connect(system): #systems are the arrays with objects with have .IP and .port (robots, plants)
    for item in system:
        sockets.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
        sockets[-1].connect((item.IP, item.port))

### main ###
if __name__ == "__main__":
    # initialize / get tag locations
#     while (initAnchors(camera_port, anchors) == False):
#         continue
    initAnchors(camera_port, anchors)
    
#     updTagPos(camera_port, plants, anchors, field_width)
#     updTagPos(camera_port, robots, anchors, field_width, True)
    
    # intial calculation of closest robot targets (minimal movement)
    # use Hungarian algorithm (O(n^3)): uses cost matrix to maximize efficiency 

#     #initialize cost matrix
#     matrix = []
#     row = 0
#     for robot in robots:
#         robot.setState("Ice")
#         matrix.append([])
#         for ice in ice_patches:
#             matrix[row].append((math.sqrt((robot.coords.x-ice.coords.x)**2 + (robot.coords.y-ice.coords.y)**2)))
#         row+=1
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
        
    while True:
        print(str(anchors[0].coords.x) + " " + str(anchors[0].coords.y))
#         for i in range(len(robots)):
#             if robots[i].changeState(): # if we have arrived at target, we want to set a new target
#                 setNewTarget(robots[i])
#             velocity = get_velocity(robots[i], obstacles, alpha=1.0, speed=0.10, obs_clearance=0.01, detect_clearance=0.10)
#             write(sockets[i], "vx: " + velocity[0] + ", vy: " + velocity[1])
#         updRobotPos(camera_port, robots, anchors, field_width)
#         print(str(robots[0].coords.x) + " " + str(robots[0].coords.y))  #testing
    
