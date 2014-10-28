import numpy as np
import cv2

cap = cv2.VideoCapture(0)

fgbg = cv2.BackgroundSubtractorMOG2()

while(1):
    ret, frame = cap.read()

    fgmask = fgbg.apply(frame)
    
    # blur = cv2.GaussianBlur(fgmask,(5,5),0)
    # ret,thresh1 = cv2.threshold(blur,70,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # hull = cv2.convexHull(fgmask)
    
    cv2.imshow('frame',thresh1)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
