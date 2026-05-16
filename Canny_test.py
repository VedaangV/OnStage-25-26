from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np
import threading
 
max_value = 255
low_canny = 0
high_canny = max_value
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_canny_name = 'Canny Low Threshold'
high_canny_name = 'Canny High Threshold'
 
 
def on_low_canny_thresh_trackbar(val):
    global low_canny
    global high_canny
    low_canny = val
    low_canny = min(high_canny-1, low_canny)
    cv.setTrackbarPos(low_canny_name, window_detection_name, low_canny)
 
 
 
def on_high_canny_thresh_trackbar(val):
    global low_canny
    global high_canny
    high_canny = val
    high_canny = max(high_canny, low_canny+1)
    cv.setTrackbarPos(high_canny_name, window_detection_name, high_canny)
 
parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()
 
### setup camera ###
class VideoStream:
    def __init__(self):
        gst_pipeline = (
            "souphttpsrc location=http://192.168.32.214:8080/video is-live=true ! "
            "multipartdemux ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
        self.cap = cv.VideoCapture(gst_pipeline, cv.CAP_GSTREAMER)
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
 
cap = VideoStream()
 
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)
 
 
 
cv.createTrackbar(low_canny_name, window_detection_name , low_canny, max_value, on_low_canny_thresh_trackbar)
cv.createTrackbar(high_canny_name, window_detection_name , high_canny, max_value, on_high_canny_thresh_trackbar)
 
while True:
    
    ret, frame = cap.read()
    if frame is None:
        break
    
    h, w, c = frame.shape
    
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray_blurred = cv.GaussianBlur(frame_gray, (3,3), 0)
    frame_threshold = cv.Canny(frame_gray, low_canny, high_canny) #160, 255
    
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    frame_threshold = cv.morphologyEx(frame_threshold, cv.MORPH_CLOSE, kernel)
    
    contours, hierarchy = cv.findContours(frame_threshold, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    ctrs = np.zeros((h, w, 1), dtype=np.uint8)
    ctrs = np.zeros((h, w, 1), dtype=np.uint8)
    cv.drawContours(ctrs, contours, -1, 255)
    
    for idx in range(len(contours)):
        print(f"{idx}: {cv.contourArea(contours[idx]):.2f}")
        
    cv.imshow(window_capture_name, frame_gray_blurred)
    cv.imshow(window_detection_name, ctrs)
    
 
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break

