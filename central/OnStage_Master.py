### import libraries ###
import numpy as np
import math

### import subfiles ###
from CBF import drive
from OnStage_Rcoords import updatePositions
from WifiComms import write, read

states = ["plant", "ice", "mining"]

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

def connect():
    for robot in robots:
        sockets.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
        sockets[-1].connect((robot.IP, robot.port))

### main ###
if __name__ == "__main__":
    while True:
        for idx in range(len(robots)):
            robot = robots[idx]
            robot.changeState()
            velocity = drive(robot, np.array(robot.targetX, robot.targetY), obstacles, alpha=1.0, speed=0.10, obs_clearance=0.01, detect_clearance=0.10)
            write(sockets[idx], "vx: " + velocity[0] + ", vy: " + velocity[1])
        updatePositions()
        
        
        
    
    
    
    
    
    
