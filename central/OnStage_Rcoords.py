### INFO ###
# install apriltag library by creating a virtual environment and pip install
# make sure to change "module_dir" to match path where library is located

# change "camera_type" to match camera being used e.x. "picam", "ausdom", "logitech"...
# change "camera_port" to match camera usb port
#    can be checked using linux commands
#    sudo apt install v4l-utils
#    v4l2-ctl --list-devices

### import subfiles ###
from OnStage_Common import *

options = apriltag.DetectorOptions(families="tag36h11") #setup AT
detector = apriltag.Detector(options)

os.environ["OPENCV_LOG_LEVEL"] = "OFF"

### functions ###
def convertPos(original_coords, anc_topleft, anc_topright, anc_bottomleft):
    new_coords = Point(FIELD_WIDTH*((anc_topleft.x - original_coords.x) / (anc_topleft.x - anc_topright.x)), FIELD_LENGTH*((anc_bottomleft.y - original_coords.y) / (anc_bottomleft.y - anc_topleft.y)))
    return new_coords

def displayTags(results, img):
    ### draw bounding boxes and info of ATs ###
    for r in results:
        # get points
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw lines
        cv2.line(img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(img, ptD, ptA, (0, 255, 0), 2)
        # draw center
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
        # draw tag family
        tagFamily = r.tag_family.decode("utf-8")
    return img
    
def initAnchors(cam, anchors):
    ### get image as RGB and GRAY ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        return -1
    rgb = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
    results = detector.detect(gray)

    ### update robot x and y values ###
    tags_detected = 0
    for r in results:
        for i in range(len(anchors)):
            if (anchors[i].tag == r.tag_id):
                tags_detected = tags_detected + 1
                anchors[i].coords = Point(r.center[0], r.center[1])
                break
    
    rgb = displayTags(results, rgb)
    if tags_detected == len(anchors):
        return 1, rgb
    return 0, rgb
    
def updTagPos(cam, group, anchors):
    ### get image as RGB and GRAY ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        return -1, None
    rgb = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
    results = detector.detect(gray)
    
    ### update robot x and y values ###
    tags_detected = 0
    for r in results:
        for i in range(len(group)):
            if (group[i].tag == r.tag_id):
                tags_detected = tags_detected + 1
                p = Point(r.center[0], r.center[1])
                p = convertPos(p, anchors[0].coords, anchors[1].coords, anchors[2].coords)
                if p.x > FIELD_WIDTH:
                    p.x = FIELD_WIDTH
                if p.x < 0:
                    p.x = 0
                if p.y > FIELD_LENGTH:
                    p.y = FIELD_LENGTH
                if p.y < 0:
                    p.y = 0
                group[i].coords = p
                break
            
    rgb = displayTags(results, rgb)
    if tags_detected == len(group):
        return 1, rgb
    return 0, rgb

def updObsPos(cam, obstacles, anchors):
    ### get image as RGB ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        return -1

    h, w, c = frame.shape
    
    ### canny edge detection ###
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray_blurred = cv2.GaussianBlur(frame_gray, (5,5), 5)
    frame_threshold = cv2.Canny(frame_gray, CANNY_LBOUND, CANNY_UBOUND) #160, 255
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    frame_threshold = cv2.dilate(frame_threshold, kernel, iterations=1)
    frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_CLOSE, kernel)
    
    ### get contours ###
    contours, hierarchy = cv2.findContours(frame_threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    ### filter based on area ###
    temp = []
    for idx in range(len(contours)):
        if cv2.contourArea(contours[idx]) >= AREA_MINIMUM and cv2.contourArea(contours[idx]) < AREA_MAXIMUM:
            temp.append(contours[idx])
    contours = temp
    
    # image for displaying contours 
    ctrs = np.zeros((h, w, 1), dtype=np.uint8)
    ctrs = np.zeros((h, w, 1), dtype=np.uint8)
    cv2.drawContours(ctrs, contours, -1, 255)
    
    obstacle_count = 0
    if (len(contours) == 0):
        return 0, ctrs
    for idx, cnt in enumerate(contours):
        if len(obstacles) == obstacle_count:
            obstacles.append(copy.deepcopy(obstacles[0]))
            obstacles[-1].coords.x = -1
            obstacles[-1].coords.y = -1
            obstacles[-1].border = []
            obstacles[-1].radius = 0
            
        M = cv2.moments(cnt)
        
        ### .coords = centoid ###
        if M["m00"] != 0:
            centroid = Point(int(M['m10']/M['m00']), int(M['m01']/M['m00']))
        else:
            centroid = Point(0, 0)
            
#         if centroid.x < anchors[0].coords.x or centroid.x > anchors[1].coords.x or centroid.y < anchors[0].coords.y or centroid.y > anchors[2].coords.y:
#             continue
    
        centroid = convertPos(centroid, anchors[0].coords, anchors[1].coords, anchors[2].coords)
        obstacles[obstacle_count].coords = centroid
        
        ### .radius = average of max and min dist from centoid along contour ###
        points = cnt.reshape(-1, 2)
        minDist = FIELD_WIDTH + FIELD_LENGTH
        maxDist = 0
        for i in range(len(points)):
            point = Point(points[i][0], points[i][1])
            point = convertPos(point, anchors[0].coords, anchors[1].coords, anchors[2].coords)
            if (point.distance_to(centroid) < minDist):
                minDist = point.distance_to(centroid)
            if (point.distance_to(centroid) > maxDist):
                maxDist = point.distance_to(centroid)
            ### .border = points along edge of contour ###
            obstacles[obstacle_count].border.append([point.x, point.y]) ### 
        if (minDist == 80 or maxDist == 0):
            return -1
        obstacles[obstacle_count].radius = (minDist + maxDist) / 2
        
        print(f"{obstacle_count}: radius-{obstacles[obstacle_count].radius:.2f}, area-{cv2.contourArea(cnt):.2f}") #testing#
        obstacle_count = obstacle_count + 1
    
    if obstacle_count > 0:
        return 1, ctrs
    return 0, ctrs

def updBasePos(cam, base, anchors):
    ### get image as RGB and GRAY ###
    ret, frame = cam.read()
    if not ret:
        print("Failed to get camera frame")
        return -1, None
    rgb = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
    results = detector.detect(gray)
    
    ### update robot x and y values
    for r in results:
        if (base.tag == r.tag_id):
            p = Point(r.center[0], r.center[1])
            (ptA, ptB, ptC, ptD) = r.corners
            ptA = Point(int(ptA[0]), int(ptA[1]))
            
            p = convertPos(p, anchors[0].coords, anchors[1].coords, anchors[2].coords)
            ptA = convertPos(ptA, anchors[0].coords, anchors[1].coords, anchors[2].coords)
            angle_rad = math.atan2((p.y - ptA.y), (p.x-ptA.x)) - math.pi/4
            
            print(angle_rad)
            
            p1 = Point(p.x + 0.7, p.y + 0.75)
            
            p2 = Point(p.x + 1.3, p.y + 0.75)
            
            base.coords = p
            base.entrances[0].coords = p1
            base.entrances[1].coords = p2
            break
            
    rgb = displayTags(results, rgb)
    if base.coords.x == -1 and base.coords.y == -1:
        return 0, rgb
    else:
        return 1, rgb
