import cv2
import numpy as np

cap = cv2.VideoCapture(0)

def find_ice(frame: np.ndarray):
    h, w, c = frame.shape

    # lighting normalization
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    # adaptive contrast
    clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(4, 4))
    l = clahe.apply(l)

    lab = cv2.merge((l,a,b))
    img_norm = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    hsv = cv2.cvtColor(img_norm, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv, (90, 50, 50), (130, 255, 255))

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ctrs = np.zeros((h, w, 1), dtype=np.uint8)
    cv2.drawContours(ctrs, contours, -1, 255)
    cv2.imshow("Contours", ctrs)
    cv2.imshow("Filtered", img_norm)

while True:
    ret, frame = cap.read()
    find_ice(frame);
    key = cv2.waitKey(1);
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
