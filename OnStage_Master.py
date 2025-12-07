import cv2
import socket
import time
import apriltag
import math
import numpy as np
cap = cv2.VideoCapture(0)  # 0 = default USB webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


states = ["plant", "ice", "mining"]


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

def updatePositions():
    #updates x and y for each robot and plant based on April Tags
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   
    # Detect AprilTags
    results = detector.detect(gray)
    print("[INFO] {} AprilTags detected".format(len(results)))

    for r in results:
        # Extract corners
        (ptA, ptB, ptC, ptD) = r.corners
        ptA = tuple(map(int, ptA))
        ptB = tuple(map(int, ptB))
        ptC = tuple(map(int, ptC))
        ptD = tuple(map(int, ptD))

        # Draw bounding box
        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

        # Draw center
        cX, cY = map(int, r.center)
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

        # Draw tag ID
        tagFamily = r.tag_family.decode("utf-8")
        #cv2.putText(frame, f"{r.tag_id} {tagFamily}", (ptA[0], ptA[1]-10),
                    #cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print(f"\ttag_id: {r.tag_id}, tag_family: {tagFamily}")
        ID = min(r.tag_id, 2)
        if(ID < 10 ):
            robots[ID].updatePosition(cX, cY)
        else:
            plants[ID-10].updatePosition(cX, cY)
    # Show the image
    #cv2.imshow("AprilTag Detection", frame)

    # Press 'q' to quit


if __name__ == "__main__":
    while True:
        updatePositions();
        for robot in robots:
            print(f"robot {robot.tag}: ({robot.x}, {robot.y})")
    
    
    
    
    
    