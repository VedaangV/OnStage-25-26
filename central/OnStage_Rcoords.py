### INFO ###
# install apriltag library by creating a virtual environment and pip install
# make sure to change "module_dir" to match path where library is located

# change "camera_type" to match camera being used e.x. "picam", "ausdom", "logitech"...
# change "camera_port" to match camera usb port
#    can be checked using linux commands
#    sudo apt install v4l-utils
#    v4l2-ctl --list-devices
# tune additional camera values depending on testing environment (camera_brightness, camera_contrast...)

### import libraries ###
import cv2
import apriltag
import numpy as np

from picamera2 import Picamera2

module_dir = os.path.abspath('/home/pi/onstage/python/apriltag/lib/python3.11/site-packages')
sys.path.append(module_dir)
import apriltag

### setup camera ###
camera_type = "ausdom"
camera_port = 8
camera_width = 640; camera_height = 480
camera_brightness = 0; camera_contrast = 0

if camera_type == "picam":
    cam = Picamera2()
    config = cam.create_preview_configuration(lores={"size": (camera_width, camera_height)}) #(640, 480)
    cam.configure(config)
    cam.start()
else:
    cam = cv2.VideoCapture(camera_port)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    if not cam.isOpened():
        print("Failed to open camera")
        exit()

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

### functions ###
def apply_imgfx(input_img, brightness = 0, contrast = 0):
    if brightness != 0:
        if brightness > 0:
            shadow = brightness
            highlight = 255
        else:
            shadow = 0
            highlight = 255 + brightness
        alpha_b = (highlight - shadow)/255
        gamma_b = shadow
        
        buf = cv2.addWeighted(input_img, alpha_b, input_img, 0, gamma_b)
    else:
        buf = input_img.copy()
    
    if contrast != 0:
        f = 131*(contrast + 127)/(127*(131-contrast))
        alpha_c = f
        gamma_c = 127*(1-f)
        
        buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)

    return buf

def displayTags(img, results):
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
        cv2.putText(img, (str(r.tag_id) + " " + tagFamily), (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    ### display edited image ###
    cv2.imshow("Camera", rgb)
    return

def updTagPos(robots):
    ### get image as RGB and GRAY ###
    if camera_type == "picam":
        yuv420 = cam.capture_array("lores")
        rgb = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2RGB)
        gray = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2GRAY)
    else:
        ret, frame = cam.read()
        if not ret:
            print("Failed to get camera frame")
            break
        frame = apply_imgfx(frame, camera_brightness, camera_contrast) 
        rgb = frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ### get AT detection results ###
    options = apriltag.DetectorOptions(families="tag36h11") #setup AT
    detector = apriltag.Detector(options) #setup AT
    results = detector.detect(gray)

    ### update robot x and y values
    for r in results:
        for robot in robots:
            if (robot.tag == r.tag_id):
                robot.x = r.center[0]
                robot.y = r.center[1]
                break
            
    displayTags(rgb, results) #
    return
