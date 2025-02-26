import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3, 640) #width
cap.set(4, 480) #height
cap.set(10, 20) #brightness

def findContours(img, copy):
    area, peri = 0, 0
    x, y, w, h = 0, 0, 0, 0
    #contours is a list of all shapes found in a frame
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            cv2.drawContours(copy, cnt, -1, (0,255,0), 2)
            peri = cv2.arcLength(cnt, True) #True indicates the shape is closed
            #Approximate the number of corners of the shape
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x,y,w,h = cv2.boundingRect(approx) #Bounding rectangle for reduced points in countour
    return area, peri, x, w, y, h
    

#In HSV
colorParams = [136, 52, 94, 179, 255, 255] #test color

def findColor(image, copy):
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    lower = np.array([129, 57, 187])
    upper = np.array([179, 255, 255])
    mask = cv2.inRange(imgHSV, lower, upper) #create a mask with an accepted range of HSV values
    a, p, x, w, y, h = findContours(mask, copy) #returns bounding box coordinates
    cv2.rectangle(copy, (x, y), (x + w, y + h), (0, 255, 0), 5) 
    cv2.putText(copy, "Ball",(x + (w // 2) + 10, y + h - 10), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0), 2)
    print(f"Area: {a} Perimeter: {p}")
    



while True:
    success, img = cap.read()
    img_copy = img.copy()

    findColor(img, img_copy)

    cv2.imshow("Test", img_copy)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break