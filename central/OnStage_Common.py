### libraries ###
import sys
import os
import numpy as np
import math
import re
import subprocess
import threading
import socket
import time
import copy

import cv2
module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag

from munkres import Munkres, print_matrix

from OnStage_Audio import *
from OnStage_WifiComms import *

### constants ###
ENABLE_WIFI = True
FIELD_WIDTH = 3.5 #ft
FIELD_LENGTH = 3.5 #ft

#_Common.py
ICE_LEVEL = 4
PLANT_LEVEL = 4

#_Master.py
CBF_GAMMA = 0.7 #CBF aggressiveness — raise if robots get too close to obstacles/each other
CBF_KATT = 1.2 #waypoint attraction strength
CBF_SAFETYMARGIN = 0.37 #extra clearance in field units (on top of robot + obstacle radii)
CAMERA_TYPE = "usb"
CAMERA_CONTRAST = 1.0
CAMERA_BRIGHTNESS = -40

#_CBF.py
ROBOT_RADIUS     = 0.37 #ft
DIST_THRESHOLD   = 0.8 #ft #how from target to count as successful
MAX_SPEED        = 0.4 #ft/s
MIN_SPEED        = 0.4 #ft/s

#_Rcoords.py
CANNY_LBOUND = 160
CANNY_UBOUND = 255
AREA_MINIMUM = 5 #minimum area of contour to be considered an obstacle
AREA_MAXIMUM = 10000 #maximum area of contour to be considered an obstacle

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
    tag: int
    IP: str
    port: int
    sock = None
    coords: Point = Point(-1, -1)
    target: None
    state: str = "None"
    haswater: bool = False
    
    Vx = 0.0
    Vy = 0.0
    Vx_act = 0.0
    Vy_act = 0.0
    
    def __init__(self, IP, port, tag):
        self.IP = IP;
        self.port = port
        self.tag = tag;
    def setTarget(self, target):
        self.target = target
    def collectWater(self):
        self.haswater = True
        if ENABLE_WIFI == True:
            wifi_write(self.sock, "collect")
    def depleteWater(self):
        self.haswater = False
        if ENABLE_WIFI == True:
            wifi_write(self.sock, "deplete")
    
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
        if (self.level < PLANT_LEVEL):
            play_watering()
            self.level += 1
            if ENABLE_WIFI == True:
                wifi_write(self.sock, "G")

class ice:
    tag: int
    IP: str
    port: int
    sock = None
    coords: Point = Point(-1, -1)
    level: int = ICE_LEVEL
    available: bool = True
    
    def __init__(self, IP, port, tag):
        self.IP = IP
        self.port = port
        self.tag = tag
    def deplete(self):
        if (self.level > 0):
            play_mining()
            self.level -= 1
            if ENABLE_WIFI == True:
                wifi_write(self.sock, "D")
