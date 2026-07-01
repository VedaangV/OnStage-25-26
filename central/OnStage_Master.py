### import subfiles ###
from OnStage_Common import *
from OnStage_Rcoords import updTagPos, updObsPos, updBasePos, initAnchors
from OnStage_CBF import CBFController, followPath

###***** CLASS ARRAYS, CHANGE DEPENDING ON SETUP *****###
#

robots = [robot("10.42.0.47", 5000, 0), robot("10.42.0.122", 5000, 5)]
anchors = [anchor(1), anchor(2), anchor(3)]  #AT tag 0-2

obstacles = [obstacle()]
plants = [plant("10.42.0.169", 80, 7), plant("10.42.0.140", 80, 8)]
icepatches = [ice("10.42.0.163", 81, 4), ice("10.42.0.61", 81, 6)]
base = base("10.42.0.213", 80, 9)

# robots = [robot("192.168.32.152", 5000, 0), robot("192.168.32.243", 5000, 5)]
# anchors = [anchor(1), anchor(2), anchor(3)]  #AT tag 0-2
# 
# obstacles = [obstacle()]
# plants = [plant("192.168.32.231", 80, 8), plant("192.168.32.209", 80, 7)]
# icepatches = [ice("192.168.32.118", 81, 4), ice("192.168.32.171", 81, 6)]#, ice("192.168.32.118", 81, 4)]
# base = base("192.168.32.136", 80, 9)

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
        
# Camera object
cam = VideoStream()
# ImageZMQ from Jetson to PC
sender = imagezmq.ImageSender(connect_to=f"tcp://{PC_IP}:5555")
jetson_name = "jetson"

def displayElements(img, anchors, robots, obstacles, base):
    for robot in robots:
        startpt = (int(robot.coords.x / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - robot.coords.y / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y))))
        endpt = (int((robot.coords.x + robot.Vx) / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - (robot.coords.y + robot.Vy) / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y))))
        cv2.arrowedLine(img, startpt, endpt, (0, 255, 255), 3)
        
        endpt = (int((robot.coords.x + robot.Vx_act) / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - (robot.coords.y + robot.Vy_act) / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y))))
        cv2.arrowedLine(img, startpt, endpt, (0, 100, 255), 3)
    
        if hasattr(robot, "target") and hasattr(robot.target, "coords"):
            cv2.circle(img, (int(robot.target.coords.x / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - robot.target.coords.y / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y)))), 5, (255, 255, 0), -1)    

    for obs in obstacles:
        pts = np.array(obs.border)
        for i in range(len(pts)):
            pts[i][0] = pts[i][0] / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x
            pts[i][1] = anchors[2].coords.y - pts[i][1] / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y))
        pts = pts.astype(np.int32)
        cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=2)
    
    if base is not None:
        for entrance in base.entrances:
            cv2.circle(img, (int(entrance.coords.x / FIELD_WIDTH * (abs(anchors[0].coords.x - anchors[1].coords.x)) + anchors[0].coords.x), int(anchors[2].coords.y - entrance.coords.y / FIELD_LENGTH * (abs(anchors[0].coords.y - anchors[2].coords.y)))), 5, (255, 0, 150), -1)
        
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
            want_plant[row].target = freeplant[column] 
            freeplant[column].available = False
            want_plant[row].state = "Plant"
        for r in want_plant:
            if r.state != "Plant":
                r.state = "Waiting"
    return

def asyncDisplay(window_name, frame):
    if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
        cv2.imshow(window_name, frame)
        stop = cv2.waitKey(1) & 0xFF == ord('q')
    else:
        reply = sender.send_image(jetson_name, frame)
        reply = reply.decode('utf-8')
        if reply == "STOP":
            stop = True
        else:
            stop = False
    return stop
            
### main ###
async def main():
    if ENABLE_SOUND == True:
        play_bg() ### start the background music
    
    ### position camera ###
    while True:
        ret, frame = cam.read()
        if not ret:
            print("Failed to get camera frame")
            continue
        
        if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
            cv2.imshow("Setup", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            reply = sender.send_image(jetson_name, frame)
            reply = reply.decode('utf-8')
            if reply == "STOP":
                break
    if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
        cv2.destroyAllWindows()
    
    ### initialize wifi ###
    if ENABLE_WIFI == True:
        for robot in robots:
            robot.reader, robot.writer = await wifi_connect(robot.IP, robot.port)
        for plant in plants:
            plant.reader, plant.writer = await wifi_connect(plant.IP, plant.port)
        for ice in icepatches:
            ice.reader, ice.writer = await wifi_connect(ice.IP, ice.port)
        if base is not None:
            base.reader, base.writer = await wifi_connect(base.IP, base.port)
            
    while True:
        ret, frame = cam.read()
        if not ret:
            print("Failed to get camera frame")
            continue
        
        if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
            cv2.imshow("Setup", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            reply = sender.send_image(jetson_name, frame)
            reply = reply.decode('utf-8')
            if reply == "STOP":
                break
    if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
        cv2.destroyAllWindows()
            
    ### initialize tags/positions ###
    # anchors
    while (res := initAnchors(cam, anchors))[0] != 1:
        for i in range(len(anchors)):
            print(f"Anchor {i}: {anchors[i].coords.x:.2f} {anchors[i].coords.y:.2f}") #testing#
        print("") #testing#
        img = res[1]
        if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
            cv2.imshow("Testing", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            reply = sender.send_image(jetson_name, frame)
            reply = reply.decode('utf-8')
            if reply == "STOP":
                break
    
    # plants, ice, robots
    while(res := updTagPos(cam, plants, anchors))[0] != 1:
        for plant in plants:
            print(f"Plant AT{plant.tag}: {plant.coords.x:.2f} {plant.coords.y:.2f}") #testing#
        print("")
        img = res[1]
        if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
            cv2.imshow("Testing", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            reply = sender.send_image(jetson_name, frame)
            reply = reply.decode('utf-8')
            if reply == "STOP":
                break
            
    while(res := updTagPos(cam, icepatches, anchors))[0] != 1:
        for ice in icepatches:
            print(f"Ice AT{ice.tag}: {ice.coords.x:.2f} {ice.coords.y:.2f}")
        print("")
        img = res[1]
        if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
            cv2.imshow("Testing", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            reply = sender.send_image(jetson_name, frame)
            reply = reply.decode('utf-8')
            if reply == "STOP":
                break
            
    while(res := updTagPos(cam, robots, anchors))[0] != 1:
        for robot in robots:
            print(f"Robot AT{robot.tag}: {robot.coords.x:.2f} {robot.coords.y:.2f}") #testing#
        print("")
        img = res[1]
        if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
            cv2.imshow("Testing", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            reply = sender.send_image(jetson_name, frame)
            reply = reply.decode('utf-8')
            if reply == "STOP":
                break
            
    if base is not None:
        while(res := updBasePos(cam, base, anchors))[0] != 1:
            print(f"Base AT{base.tag}: {base.coords.x:.2f} {base.coords.y:.2f}") #testing#
            print("")
            img = res[1]
            if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
                cv2.imshow("Testing", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                reply = sender.send_image(jetson_name, frame)
                reply = reply.decode('utf-8')
                if reply == "STOP":
                    break

    # obstacles
    while (res := updObsPos(cam, obstacles, anchors))[0] != 1:
        img = res[1]
        if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
            cv2.imshow("Testing", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            reply = sender.send_image(jetson_name, frame)
            reply = reply.decode('utf-8')
            if reply == "STOP":
                break
    
    i = 0
    for r in robots:
        i = 0
        while (i < len(obstacles)):
            if obstacles[i].coords.distance_to(r.coords) < 0.6:
                obstacles.pop(i)
            else:
                i = i + 1
                
    for ice in icepatches:
        await ice.reset()
    for plant in plants:
        await plant.reset()
    if base is not None:
        await base.reset()
    
    print(f"Base: {base.coords.x:.2f}, {base.coords.y:.2f}")
    print(f"Base: {base.entrances[0].coords.x:.2f}, {base.entrances[0].coords.y:.2f}")
    print(f"Base: {base.entrances[1].coords.x:.2f}, {base.entrances[1].coords.y:.2f}")
    input("Press Enter to start: ")
    if ("imshow".casefold() in DISPLAY_TYPE.casefold()) or ("local".casefold() in DISPLAY_TYPE.casefold()) or ("hdmi".casefold() in DISPLAY_TYPE.casefold()):
        cv2.destroyAllWindows()
    
    assignTargets(robots, icepatches, plants)
    
    loop = asyncio.get_running_loop()
    window_name = "Camera Display"
    
    start_time = time.perf_counter()
    
    ### loop 1 ###
    while True:
        current_time = time.perf_counter()
        if (current_time - start_time > DUSTSTORM_ACTIVATION_TIME):
            break
        
        err, img = updTagPos(cam, robots, anchors)
        
        for robot in robots:
            if robot.state == "None" or robot.state == "Waiting":
                await robot.stop()
            else:
                finished = await followPath(cbf, robot, robots, obstacles)
                if finished == True:
                    await robot.stop()
                    if (robot.state == "Ice"):
                        await robot.target.deplete()
                        if robot.target.level == 0:
                            robot.target.available = False
                        else:
                            robot.target.available = True
                        await robot.collectWater()
                    if (robot.state == "Plant"):
                        await robot.target.grow()
                        if robot.target.level == PLANT_LEVEL:
                            robot.target.available = False
                        else:
                            robot.target.available = True
                        await robot.depleteWater()
                    robot.state = "None"
                    robot.target = None
                    assignTargets(robots, icepatches, plants)
        
        if (err != -1):
            img = displayElements(img, anchors, robots, obstacles, base)
            
            stop = await loop.run_in_executor(None, asyncDisplay, window_name, img)
            if stop == True:
                break
            
    ### dust storm ###
    if base is not None:
        for robot in robots:
            await robot.dustStorm()

        for plant in plants:
            plant.available = True
        for ice in icepatches:
            ice.available = True
        
        await base.dustStorm()
        
        for idx, robot in enumerate(robots):
            robot.target = base.entrances[idx]
            robot.state = "Base"
        
        while True:
            if all(robot.state == "None" for robot in robots):
                break
            
            for robot in robots:
                err, img = updTagPos(cam, robots, anchors)
                
                if robot.state == "None" or robot.state == "Waiting":
                    await robot.stop()
                else:
                    finished = await followPath(cbf, robot, robots, obstacles)
                    
                    if finished == True:
                        robot.state = "None"
                        robot.target = None
                        await robot.stop()
                            
                if (err != -1):
                    img = displayElements(img, anchors, robots, obstacles, base)
                    
                    stop = await loop.run_in_executor(None, asyncDisplay, window_name, img)
                    if stop == True:
                        break
        
        for robot in robots:
            await robot.stop()
        await asyncio.sleep(1)
        
        await asyncio.gather(*[robot.enterBase() for robot in robots])
        await asyncio.sleep(10)
        
        await asyncio.gather(*[robot.exitBase() for robot in robots])
        await asyncio.sleep(1)
        
        await base.reset()
    
    ### loop 2 ###
    assignTargets(robots, icepatches, plants)
    
    while not (all(ice.level == 0 for ice in icepatches) and all(plant.level == PLANT_LEVEL for plant in plants)):
        err, img = updTagPos(cam, robots, anchors)
        
        for robot in robots:
            if robot.state == "None" or robot.state == "Waiting":
                await robot.stop()
            else:
                finished = await followPath(cbf, robot, robots, obstacles)
                if finished == True:
                    await robot.stop()
                    if (robot.state == "Ice"):
                        await robot.target.deplete()
                        if robot.target.level == 0:
                            robot.target.available = False
                        else:
                            robot.target.available = True
                        await robot.collectWater()
                    if (robot.state == "Plant"):
                        await robot.target.grow()
                        if robot.target.level == PLANT_LEVEL:
                            robot.target.available = False
                        else:
                            robot.target.available = True
                        await robot.depleteWater()
                    robot.state = "None"
                    robot.target = None
                    assignTargets(robots, icepatches, plants)
        
        if (err != -1):
            img = displayElements(img, anchors, robots, obstacles, base)
            
            stop = await loop.run_in_executor(None, asyncDisplay, window_name, img)
            if stop == True:
                break
    
    cv2.destroyAllWindows()
    cam.stop()

if __name__ == "__main__":
    asyncio.run(main())
