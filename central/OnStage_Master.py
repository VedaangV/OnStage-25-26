### import libraries ###
import numpy as np
import math

### import subfiles ###
from CBF import get_velocity
from OnStage_Rcoords import updRobotPos, initPlantPos
from WifiComms import write, read

states = ["ice", "deposit", "withdraw", "plant"]  #!tentative #deposit and withdraw refer to moving water in and out of base station

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
        self.haswater = False
        self.state = "ice"
        self.tag = tag;
    def setTarget(x, y):
        self.target = Point(x, y)
    def changeWater():
        self.haswater = not self.haswater
    def atTarget():
        return (math.sqrt((self.x-self.targetX)**2 + (self.y-self.targetY)**2) < 0.5)  #!tentative
    
    def setState(int s): #set state manually
        self.state = states[s]
    def changeState(): #change state after completing current task
        if (self.atTarget == True):
            self.changeWater()
            for i in range(len(states)):
                if (self.state == states[i]):
                    self.state == states[(i + 1) % len(states)]

class obstacle:
    def __init(self, x, y, radius):
        self.coords = Point(x, y)
        self.radius = radius

class plant:
    def __init__(self, IP, port, x, y, level):
        self.IP = IP;
        self.port = port
        self.coords = Point(x, y)
        self.level = level
        
class ice:
    def __init__(self, x, y, level):
        self.coords = Point(x, y)
        self.level = level

robots = [robot("192.168.32.172", 5000, 0, 0, 2), robot("192.168.32.172", 5000, 0, 0, 3)] 
anchors = [Point(0, 0), Point(0, 0)]  #AT tag 0 and 1 respectively
obstacles = [obstacle(0, 0, 0), obstacle(0, 0, 0), obstacle(0, 0, 0)]
plants = [plant("<INSERT IP HERE>", 0, 0, 0, 0)]

field_width = 8  #scales x dimension of relative coordinates
field_length = 8 #scales y dimension of relative coordinates
robotcount = 2  #predefined number of robots (prevent detection of extraneous tags)

sockets = []

def connect(system): #systems are the arrays with objects with have .IP and .port (robots, plants)
    for item in system:
        sockets.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
        sockets[-1].connect((item.IP, item.port))

### main ###
if __name__ == "__main__":
    initPlantPos(plants);
    while True:
        for idx, robot in enumerate(robots)):
            robot.changeState()
            velocity = get_velocity(robot, obstacles, alpha=1.0, speed=0.10, obs_clearance=0.01, detect_clearance=0.10)
            write(sockets[idx], "vx: " + velocity[0] + ", vy: " + velocity[1])
        updRobotPos(robots)
        
#Point(field_width*(abs(anchors[0].x - r.center[0]) / abs(anchors[0].x - anchors[1].x)), field_length*(abs(anchors[0].y - r.center[1]) / abs(anchors[0].y - anchors[1].y)))
        
        
    
    
    
    
    
    
