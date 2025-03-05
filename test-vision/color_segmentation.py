import cv2
import numpy as np
import imutils #computer vision tools
from imutils import contours
from scipy.spatial import distance as dist
from size_measure import clockwise_pts
from size_measure import middle

cap = cv2.VideoCapture(0)

cap.set(3, 640) #width
cap.set(4, 480) #height
cap.set(10, 20) #brightness

#In HSV
colorParams = [0,97,129,179,249,255] #most accurate HSV values for test ball (bright orange)
referenceWidth = 10 #test width

def mids(img, pm, tl, tr, br, bl): #image, pixelspermetric, topleft, topright, bottomright, bottomleft
    box = (tl, tr, br, bl)
    #midpoint between tl and tr coordinates
    (topX, topY) = middle(tl, tr)
    #midpoint between bk and br coordinates
    (bottomX, bottomY) = middle(bl, br)
    #midpoint between tl and bl
    (leftX, leftY) = middle(tl, bl)
    #midpoint between tr and br
    (rightX, rightY) = middle(tr, br)

    cv2.circle(img, (int(topX), int(topY)), 5, (0,0,255), -1)
    cv2.circle(img, (int(bottomX), int(bottomY)), 5, (0,0,255), -1)
    cv2.circle(img, (int(rightX), int(rightY)), 5, (0,0,255), -1)
    cv2.circle(img, (int(leftX), int(leftY)), 5, (0,0,255), -1)

    cv2.line(img, (int(topX), int(topY)), (int(bottomX), int(bottomY)), (255,255,255), 2)
    cv2.line(img, (int(leftX), int(leftY)), (int(rightX), int(rightY)), (255,255,255), 2)

    distTop2Bott= dist.euclidean((int(topX), int(topY)), (int(bottomX), int(bottomY)))
    distLeft2Right = dist.euclidean((int(leftX), int(leftY)), (int(rightX), int(rightY))) 

    if pm is None:
        pm = distLeft2Right / referenceWidth

    #get real size of object
    szVert = distTop2Bott / pm
    szHori = distLeft2Right / pm
    
    print(f"vert: {szVert}, hor: {szHori}")
    #FALTA PONER EN TEXTO LAS DIMENSIONES
    #cv2.putText(img, "{:.1f}cm".format(szVert), )


def findContours(img, copy):
    area, peri, radius = 0, 0, 0
    topLeft, bottomRight = (0,0), (0,0)
    #contours is a list of all shapes found in a frame
    fcontours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #fcontours = fcontours[0] if len(fcontours) == 2 else fcontours[1]
    #fcontours = imutils.grab_contours(fcontours)###
    if fcontours:
        (fcontours, _) = contours.sort_contours(fcontours)####

    ppm = None #pixels per metric ###
    for cnt in fcontours:
        area = cv2.contourArea(cnt)
        if area > 500:
            cv2.drawContours(copy, cnt, -1, (0,255,0), 2)
            peri = cv2.arcLength(cnt, True) #True indicates the shape is closed
            #Approximate the number of corners of the shape
            box = cv2.minAreaRect(cnt)
            box_pts = cv2.boxPoints(box)
            box_pts = np.array(box_pts, dtype="int")
            clockCoor = clockwise_pts(box_pts)
            cv2.drawContours(copy, [clockCoor.astype("int")], -1, (255,255,0), 2)
            for (x,y) in clockCoor:
                cv2.circle(copy, (int(x), int(y)), 5, (0,0,255), -1) #circles in edges
            mids(copy, ppm, clockCoor[0], clockCoor[1], clockCoor[2], clockCoor[3])
            topLeft = tuple(clockCoor[0])
            bottomRight = tuple(clockCoor[2])
            radius = np.sqrt(area/np.pi) 
    return radius, area, peri, topLeft, bottomRight
    
def findColor(image, copy):
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    lower = np.array(colorParams[0:3])
    upper = np.array(colorParams[3:6])
    mask = cv2.inRange(imgHSV, lower, upper) #create a mask with an accepted range of HSV values
    r, a, p, top_left, bottom_right= findContours(mask, copy) #returns bounding box, area, peri, and radius
    #cv2.rectangle(copy, top_left, bottom_right, (0, 255, 0), 5) 
    cv2.putText(copy, "Ball",(bottom_right[0], bottom_right[1]), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0), 2)
    if a > 100: # just to see test objects
        print(f"Area: {a} Perimeter: {p} Radius: {r}")
    cv2.imshow("mask", mask)

while True:
    success, img = cap.read()
    img_copy = img.copy()
    if success:
        #image preprocessing
        img_blur = cv2.GaussianBlur(img, (5, 5), 0)

        findColor(img_blur, img_copy)

        cv2.imshow("Test", img_copy)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()