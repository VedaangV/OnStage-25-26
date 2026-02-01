### import libraries ###
import numpy as np
import math

from CBF import drive
from OnStage_Rcoords import updatePositions
from WifiComms import write, read

### setup camera ###
cap = cv2.VideoCapture(0)  # 0 = default USB webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

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
        self.x = x
        self.y = x
        self.water_level = 1
        self.state = "plant"
        self.tag = tag;
    def updatePosition(self, x, y):
        self.x = x
        self.y=y
    def setTarget(x, y):
        self.targetX = x
        self.targetY = y
    def depleteWater():
        self.water_level -= 1
    def restoreWater():
        self.water_level += 1
    def atTarget():
        return (math.sqrt((self.x-self.targetX)**2 + (self.y-self.targetY)**2) < 0.5)
    
    def changeState():
        if(atTarget):
            self.state = states[(states.index(self.state)+1)%len(states)]

class plant:
    def __init__(self, IP, port, level, x, y):
        self.IP = IP;
        self.port = port
        self.level = level
        self.x = x
        self.y = y
        
robots = [robot("192.168.32.172", 5000, 0, 0, 0), robot("192.168.32.172", 5000, 0, 0, 1), robot("192.168.32.172", 5000, 0, 0, 2)]
plants = [plant("<INSERT IP HERE>", 0, 0, 0, 0)]
obstacles = []

sockets = []

def connect():
    for robot in robots:
        sockets.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
        sockets[-1].connect((robot.IP, robot.port))
    



if __name__ == "__main__":
    while True:
        for idx in range(len(robots)):
            robot = robots[idx]
            robot.changeState()
            velocity = drive(robot, np.array(robot.targetX, robot.targetY), obstacles, alpha=1.0, speed=0.10, obs_clearance=0.01, detect_clearance=0.10)
            write(sockets[idx], "vx: " + velocity[0] + ", vy: " + velocity[1])
        updatePositions()
        
        
        
    
    
    
    
    
    
