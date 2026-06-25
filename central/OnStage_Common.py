### libraries ###
import sys
import os
import numpy as np

import math
import re

import asyncio
import socket
import time
import subprocess
import threading

import copy

import cv2
module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag
import transforms3d as t3d

from munkres import Munkres, print_matrix

from OnStage_Audio import *
from OnStage_WifiComms import *

### constants ###
ENABLE_WIFI = True
ENABLE_SOUND = False
FIELD_WIDTH = 4.5 #ft
FIELD_LENGTH = 4.5 #ft

#_Common.py
ICE_LEVEL = 4
PLANT_LEVEL = 4

#_Master.py
DUSTSTORM_ACTIVATION_TIME = 40 # seconds
CBF_GAMMA = 2.0 #CBF aggressiveness — raise if robots get too close to obstacles/each other
CBF_KATT = 1.2 #waypoint attraction strength
CBF_SAFETYMARGIN = 0 #extra clearance in field units (on top of robot + obstacle radii)
CAMERA_TYPE = "usb"
CAMERA_CONTRAST = 1.0
CAMERA_BRIGHTNESS = -40
WIN_FSCRN_WIDTH = 1920
WIN_FSCRN_HEIGHT = 1080
WIN_WIDTH = 640
WIN_HEIGHT = 480

#_CBF.py
ROBOT_RADIUS     = 0.37 #ft
DIST_THRESHOLD   = 0.6 #ft #how from target to count as successful
MAX_SPEED        = 0.4 #ft/s
MIN_SPEED        = 0.4 #ft/s

#_Rcoords.py
CANNY_LBOUND = 150
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
    reader = None
    writer = None
    coords: Point = Point(-1, -1)
    target = None
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
    async def collectWater(self):
        self.haswater = True
        if ENABLE_WIFI == True:
            await wifi_write(self.writer, "collect")
    async def depleteWater(self):
        self.haswater = False
        if ENABLE_WIFI == True:
            await wifi_write(self.writer, "deplete")
    async def dustStorm(self):
        if ENABLE_WIFI == True:
            await wifi_write(self.writer, "dust")
    async def enterBase(self):
        if ENABLE_WIFI == True:
            await wifi_write(self.writer, "enter")
    async def exitBase(self):
        if ENABLE_WIFI == True:
            await wifi_write(self.writer, "exit")
    
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
    reader = None
    writer = None
    coords: Point = Point(-1, -1)
    level: int = 0
    available: bool = True
    
    def __init__(self, IP, port, tag):
        self.IP = IP
        self.port = port
        self.tag = tag
    async def grow(self):
        if (self.level < PLANT_LEVEL):
            if ENABLE_SOUND == True:
                play_watering()
            self.level += 1
            if ENABLE_WIFI == True:
                await wifi_write(self.writer, "G")
    async def reset(self):
        self.level = 0
        self.available = True
        if ENABLE_WIFI == True:
            await wifi_write(self.writer, "R")
        
class ice:
    tag: int
    IP: str
    port: int
    reader = None
    writer = None
    coords: Point = Point(-1, -1)
    level: int = ICE_LEVEL
    available: bool = True
    
    def __init__(self, IP, port, tag):
        self.IP = IP
        self.port = port
        self.tag = tag
    async def deplete(self):
        if (self.level > 0):
            if ENABLE_SOUND == True:
                play_mining()
            self.level -= 1
            if ENABLE_WIFI == True:
                await wifi_write(self.writer, "D")
    async def reset(self):
        self.level = ICE_LEVEL
        self.available = True
        if ENABLE_WIFI == True:
            await wifi_write(self.writer, "R")

class entrance:
    coords: Point = Point(-1, -1)
    available = True
    
class base:
    tag: int
    IP: str
    port: int
    reader = None
    writer = None
    coords: Point = Point(-1, -1)
    
    def __init__(self, IP, port, tag):
        self.IP = IP
        self.port = port
        self.tag = tag
    async def dustStorm(self):
        if ENABLE_WIFI == True:
            await wifi_write(self.writer, "S")
    async def reset(self):
        if ENABLE_WIFI == True:
            await wifi_write(self.writer, "T")
