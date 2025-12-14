### INFO ###
# install apriltag library by creating a virtual environment and pip install
# make sure to change module_dir to match path where library is located

### import libraries ###
import sys
import os
import cv2
import numpy as np
import copy
import math

from picamera2 import Picamera2

module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __array__(self) -> np.ndarray:
        return np.array([self.x, self.y])

### setup picam ###
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640, 480)})   #(640, 480)
picam2.configure(config)
picam2.start()

while True:
    ### get image as RGB and GRAY ###
    yuv420 = picam2.capture_array("lores")
    rgb = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2RGB)
    gray = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2GRAY)
    
    ### get results of AT detection ###
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    #print("[INFO] {} total AprilTags detected".format(len(results)))
    
    ### get coords of robot relative to cam ###
    anchors = [Point(-1, -1), Point(-1, -1)]
    robotsI = []
    for r in results:
        if r.tag_id >= 0 and r.tag_id <= 1:
            anchors[r.tag_id] = Point(r.center[0], r.center[1])
        else: #else:
            robotsI.append(Point(r.center[0], r.center[1]))
    
    ## convert coords to be relative to anchors ###
    robotsR = []
    for index, robot in enumerate(robotsI):
        if (anchors[0].x != -1 and anchors[0].y != -1 and anchors[1].x != -1 and anchors[1].y != -1):
            robotsR.append(Point((abs(anchors[0].x - robot.x) / abs(anchors[0].x - anchors[1].x)), (abs(anchors[0].y - robot.y) / abs(anchors[0].y - anchors[1].y))))
            print("Robot " + str(index) + " relative coords: " + str(robotsR[index].x) + ", " + str(robotsR[index].y))
    print("\n")
   
    ### draw bounding boxes and info of ATs ###
    for r in results:
        # get points
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw lines
        cv2.line(rgb, ptA, ptB, (0, 255, 0), 2)
        cv2.line(rgb, ptB, ptC, (0, 255, 0), 2)
        cv2.line(rgb, ptC, ptD, (0, 255, 0), 2)
        cv2.line(rgb, ptD, ptA, (0, 255, 0), 2)
        # draw center
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(rgb, (cX, cY), 5, (0, 0, 255), -1)
        # draw tag family
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(rgb, (str(r.tag_id) + " " + tagFamily), (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #print("\ttag_id: " + str(r.tag_id) + ", tag_family: {}".format(tagFamily))
    
    ### display edited image ###
    cv2.imshow("Camera", rgb)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

cv2.destroyAllWindows()
picam2.stop()
