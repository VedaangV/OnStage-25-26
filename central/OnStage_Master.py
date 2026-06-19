### import subfiles ###
from OnStage_Common import *
from OnStage_Rcoords import updTagPos, updObsPos, initAnchors
from OnStage_CBF import CBFController, followPath, stopRobot

###***** CLASS ARRAYS, CHANGE DEPENDING ON SETUP *****###
#

# robots = [robot("10.42.0.47", 5000, 0), robot("10.42.0.56", 5000, 5)]
# anchors = [anchor(1), anchor(2), anchor(3)]  #AT tag 0-2
# 
# obstacles = [obstacle()]
# plants = [plant("10.42.0.169", 80, 7), plant("10.42.0.140", 80, 8)]
# icepatches = [ice("10.42.0.163", 81, 4), ice("10.42.0.61", 81, 6)]

robots = [robot("192.168.32.152", 5000, 0)]#robot("192.168.32.242", 5000, 5)]
anchors = [anchor(1), anchor(2), anchor(3)]  #AT tag 0-2

obstacles = [obstacle()]
plants = [plant("192.168.32.231", 80, 8)]#, plant("192.168.32.209", 80, 7)]
icepatches = [ice("192.168.32.171", 81, 6)]#ice("192.168.32.120", 81, 4), ice("192.168.32.172", 81, 6)]

#
###*****     *****###

### CBF controller ###
cbf = CBFController(gamma=CBF_GAMMA, k_att=CBF_KATT, safety_margin=CBF_SAFETYMARGIN)

### setup camera ###
class VideoStream:
    def __init__(self):
        if ("phone".casefold() in CAMERA_TYPE.casefold()) or ("ipcam".casefold() in CAMERA_TYPE.casefold()):
            ### phone IPcam ###
            gst_pipeline = (
                "souphttpsrc location=http://192.168.32.214:8080/video is-live=true ! "
                "multipartdemux ! "
                "jpegdec ! "
                "videoconvert ! "
                "video/x-raw, format=BGR ! "
                "appsink drop=true max-buffers=1 sync=false"
            )
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        else:
            ### USB webcam ###
            self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            print("Failed to open camera")
            exit()

        self.ret, self.frame = self.cap.read()
        self.lock = threading.Lock()
        self.running = True

        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            # control Contrast 
            alpha = CAMERA_CONTRAST
            # control brightness
            beta = CAMERA_BRIGHTNESS
            frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta) 
            if ret:
                with self.lock:
                    self.ret = ret
                    self.frame = frame

    def read(self):
        with self.lock:
            return self.ret, self.frame

    def stop(self):
        self.running = False
        self.cap.release()
        
cam = VideoStream()

def displayElements(img, anchors, robots, obstacles):
    for robot in robots:
        startpt = (int(robot.coords.x / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - robot.coords.y / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y))))
        endpt = (int((robot.coords.x + robot.Vx) / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - (robot.coords.y + robot.Vy) / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y))))
        cv2.arrowedLine(img, startpt, endpt, (0, 255, 0), 3)
        
        endpt = (int((robot.coords.x + robot.Vx_act) / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - (robot.coords.y + robot.Vy_act) / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y))))
        cv2.arrowedLine(img, startpt, endpt, (0, 100, 255), 3)
        
        for obs in obstacles:
            pts = np.array(obs.border)
            for i in range(len(pts)):
                pts[i][0] = pts[i][0] / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x
                pts[i][1] = anchors[2].coords.y - pts[i][1] / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y))
            pts = pts.astype(np.int32)
            cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=2)
    return img
    
def assignTargets(robots, icepatches, plants): # system = plants or ice
    ### assign targets using matrix calculations ###
    # intial calculation of closest robot targets (minimal movement)
    # use Hungarian algorithm (O(n^3)): uses cost matrix to maximize efficiency 
    want_plant = []
    want_ice = []
    freeplant = []
    freeice = []
    for r in robots:
        if (r.state == "None" or r.state == "Waiting"):
            if r.haswater == False:
                want_ice.append(r)
            else:
                want_plant.append(r)
    for i in icepatches:
        if (i.available == True):
            freeice.append(i)
    for p in plants:
        if (p.available == True):
            freeplant.append(p)
    
    # ice #
    if (len(want_ice) > 0):
        matrix = []
        row = 0
        for r in want_ice:
            matrix.append([])
            for i in freeice:
                matrix[row].append(r.coords.distance_to(i.coords))
            row += 1
        
        m = Munkres()
        indexes = m.compute(matrix)
        #print_matrix(matrix, msg='Lowest cost through this matrix:')
        total = 0
        for row, column in indexes:
            want_ice[row].target = freeice[column] 
            freeice[column].available = False
            want_ice[row].state = "Ice"
        for r in want_ice:
            if r.state != "Ice":
                r.state = "Waiting"
    
    # plants #
    if (len(want_plant) > 0):
        matrix = []
        row = 0
        for r in want_plant:
            matrix.append([])
            for p in freeplant:
                matrix[row].append(r.coords.distance_to(p.coords))
            row += 1
        
        m = Munkres()
        indexes = m.compute(matrix)
        
        total = 0
        for row, column in indexes:
            print(f"working {row} {column}")
            want_plant[row].target = freeplant[column] 
            freeplant[column].available = False
            want_plant[row].state = "Plant"
        for r in want_plant:
            if r.state != "Plant":
                r.state = "Waiting"
    return

def performance():
    while True:
        err, img = updTagPos(cam, robots, anchors)
    
        for robot in robots:
            if robot.state == "None" or robot.state == "Waiting":
                stopRobot(robot)
            elif followPath(cbf, robot, robots, obstacles) == True:
                stopRobot(robot)
                if (robot.state == "Ice"):
                    robot.target.deplete()
                    if robot.target.level == 0:
                        robot.target.available = False
                    else:
                        robot.target.available = True
                    robot.collectWater()
                if (robot.state == "Plant"):
                    robot.target.grow()
                    if robot.target.level == PLANT_LEVEL:
                        robot.target.available = False
                    else:
                        robot.target.available = True
                    robot.depleteWater()
                robot.state = "None"
                robot.target = None
                assignTargets(robots, icepatches, plants)
            else:
                continue
        
        if (err != -1):
            img = displayElements(img, anchors, robots, obstacles)
            cv2.imshow("Testing", img)
            cv2.setWindowProperty("Testing", cv2.WND_PROP_TOPMOST, 1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
### main ###
if __name__ == "__main__":
    play_bg() ### start the background music
    
    ### position camera ###
    while True:
        ret, frame = cam.read()
        if not ret:
            print("Failed to get camera frame")
            continue
        cv2.imshow("Setup", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    
    ### initialize wifi ###
    if ENABLE_WIFI == True:
        for robot in robots:
            while (robot.sock == None or robot.sock == -1):
                robot.sock = wifi_connect(robot.IP, robot.port)
        for plant in plants:
            while (plant.sock == None or plant.sock == -1):
                plant.sock = wifi_connect(plant.IP, plant.port)
        for ice in icepatches:
            while (ice.sock == None or ice.sock == -1):
                ice.sock = wifi_connect(ice.IP, ice.port)
    
    ### initialize tags/positions ###
    # anchors
    while (res := initAnchors(cam, anchors))[0] != 1:
        for i in range(len(anchors)):
            print(f"Anchor {i}: {anchors[i].coords.x:.2f} {anchors[i].coords.y:.2f}") #testing#
        print("") #testing#
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # plants, ice, robots
    while(res := updTagPos(cam, plants, anchors))[0] != 1:
        for plant in plants:
            print(f"Plant AT{plant.tag}: {plant.coords.x:.2f} {plant.coords.y:.2f}") #testing#
        print("")
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    while(res := updTagPos(cam, icepatches, anchors))[0] != 1:
        for ice in icepatches:
            print(f"Ice AT{ice.tag}: {ice.coords.x:.2f} {ice.coords.y:.2f}")
        print("")
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    while(res := updTagPos(cam, robots, anchors))[0] != 1:
        for robot in robots:
            print(f"Robot AT{robot.tag}: {robot.coords.x:.2f} {robot.coords.y:.2f}") #testing#
        print("")
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # obstacles
    while (res := updObsPos(cam, obstacles, anchors))[0] != 1:
        img = res[1]
        cv2.imshow("Testing", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    i = 0
    for r in robots:
        i = 0
        while (i < len(obstacles)):
            if obstacles[i].coords.distance_to(r.coords) < 8:
                obstacles.pop(i)
            else:
                i = i + 1
    
    input("Press Enter to start: ")
    
    assignTargets(robots, icepatches, plants)
    for robot in robots:
        if (robot.state == "Ice" or robot.state == "Plant"):
            followPath(cbf, robot, robots, obstacles)
        else:
            continue
    
    ### main loop ###
    performance()
    
    cv2.destroyAllWindows()
    cam.stop()
