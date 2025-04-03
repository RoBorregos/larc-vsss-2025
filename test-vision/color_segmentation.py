
import cv2
import numpy as np
import imutils
from imutils import contours
from scipy.spatial import distance as dist
from size_measure import clockwise_pts, middle, distance
from homography import getHomography
import time

#in HSV 
colorParams = [0,103,31,20,255,255] #most accurate HSV values for test ball (bright orange) 0, 63, 255, 179, 255, 255
#checa la foto donde esta la terminal medio cubierta con los valores HSV que probaste con Alberto
#refColorParams = [0, 0, 0, 0, 0, 0]  # white 
refCenter = (320, 240) #in pixels
referenceWidth = 2.9  # Test width
ppm = None

realFieldCoors = [[0, 0], #tl
                  [150, 0], #tr
                  [150, 130], #br
                  [0, 130]] # bl

CAMERA_HEIGHT = 200 #cm
clicked_points = []

def mids(img, ppm, tl, tr, br, bl):
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

    if ppm is None:
        avrgDiameterPixels = (distLeft2Right + distTop2Bott) / 2
        ppm = avrgDiameterPixels / referenceWidth

    #get real size of object
    szVert = distTop2Bott / ppm
    szHori = distLeft2Right / ppm
    
    print(f"vert: {szVert}, hor: {szHori}")
    cv2.putText(img, f"{szVert:.1f}cm", (int(topX - 15), int(topY - 10)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(img, f"{szHori:.1f}cm", (int(rightX + 10), int(rightY)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)

def findContoursAndSize(img, copy):
    area = 0
    (cX, cY) = (0, 0)
    objCenter = 0
    #contours is a list of all shapes found in a frame
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    if not contours:
        return (0, 0)
        
    # Select biggest contour
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    cnt = contours[0] #first contour (biggest)
    
    area = cv2.contourArea(cnt)
    if area > 500:
        #Approximate the number of corners of the shape
        box = cv2.minAreaRect(cnt)
        box_pts = cv2.boxPoints(box)
        box_pts = np.array(box_pts, dtype="int")
        clockCoor = clockwise_pts(box_pts)
        cX = np.average(clockCoor[:, 0])
        cY = np.average(clockCoor[:, 1]) 
        #calculate the distance between the center and the Obj (ball in this case)
        distance(copy, refCenter, clockCoor, (cX, cY), H) #draws distances from ball's and ref's centers
        cv2.drawContours(copy, [clockCoor.astype("int")], -1, (255, 255, 0), 2) #draw ball's bounding box
        for (x, y) in clockCoor: 
            cv2.circle(copy, (int(x), int(y)), 5, (0, 0, 255), -1) # circles in edges
        mids(copy, ppm, clockCoor[0], clockCoor[1], clockCoor[2], clockCoor[3]) #mid lines in shape, includes size
        #topLeft = tuple(clockCoor[0])
        #bottomRight = tuple(clockCoor[2])
    
    return (cX, cY)

#main function
def findObject(image, copy, ppm=None): 
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Detect reference object, used section in case of going with objects rather than center of image
    # NOT in use due to the center being the reference
    '''lower_ref = np.array(refColorParams[0:3])
    upper_ref = np.array(refColorParams[3:6])
    mask_ref = cv2.inRange(imgHSV, lower_ref, upper_ref) #create a mask with an accepted range of HSV values
      
    contours_ref, _ = cv2.findContours(mask_ref, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # if there is no refObject yet
    if contours_ref and referenceObj is None: 
        contours_ref = sorted(contours_ref, key=cv2.contourArea, reverse=True)
        for cnt in contours_ref:
            area = cv2.contourArea(cnt)
            if area > 500:
                box = cv2.minAreaRect(cnt)
                box_pts = cv2.boxPoints(box)
                box_pts = np.array(box_pts, dtype="int")
                clockCoor_ref = clockwise_pts(box_pts)
                cX_ref = np.average(clockCoor_ref[:, 0])
                cY_ref = np.average(clockCoor_ref[:, 1])
                #defines the refObject
                referenceObj = distance(copy, None, clockCoor_ref, (cX_ref, cY_ref))
                cv2.drawContours(copy, [clockCoor_ref.astype("int")], -1, (255, 0, 255), 2)
                cv2.circle(copy, (int(cX_ref), int(cY_ref)), 5, (0, 255, 255), -1)
                break'''
    
    # Ball mask, already have refObj
    lower = np.array(colorParams[0:3])
    upper = np.array(colorParams[3:6])
    
    #img preprocessing
    mask = cv2.inRange(imgHSV, lower, upper)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    objCenter = findContoursAndSize(mask, copy)
    #if the center is detected, get it's coordinates in real field coordinates
    if objCenter:
        #give the correct format to the pt for use in perspectiveTransform
        objCenterPt = np.array([[objCenter[0], objCenter[1]]], dtype="float32")
        objCenterPt = np.array([objCenterPt])
        realFldCoors = cv2.perspectiveTransform(objCenterPt, H)[0][0]
        cv2.putText(copy, f"Real: ({realFldCoors[0]:.1f}, {realFldCoors[1]:.1f}) cm", (int(objCenter[0] + 50), int(objCenter[1] + 20)),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.imshow("mask", mask)
    #returns refObj(if object used) and ppm for those global variables to be in constant and correct changing state.
    return realFldCoors #regresar coordenadas REALES


cap = cv2.VideoCapture(2) #2 for external devices
cap.set(3, 640) #width
cap.set(4, 480) #height
cap.set(10, 20) #brightness

print("Homography Calibration. Click the four corners of the field in order TL, TR, BR, BL")
H = getHomography(cap, realFieldCoors)

while True:
    tpast = time.time()

    success, img = cap.read()
    img_copy = img.copy()
    if success:
        #image preprocessing
        img_blur = cv2.GaussianBlur(img, (7, 7), 0)
        
        objCoors = findObject(img_blur, img_copy, ppm)
        #objCenter are the coordinates we want for the robot to make decisions
        cv2.imshow("Test", img_copy)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(f"fps: {fps}")
        break

    #get execution time
    tnow = time.time()
    totalTime = tnow - tpast
    fps = 1 / totalTime
    #print(f"Tiempo de ejecución: {totalTime}")

cap.release()
cv2.destroyAllWindows()