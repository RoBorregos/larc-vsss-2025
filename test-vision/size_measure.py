from scipy.spatial import distance as dist #multidimensional operations provider
from imutils import perspective ####library for computer vision, perspective can order the points in tl, tr, br, bl
from imutils import contours
import numpy as np
import argparse #for console arguments **Not necessary**
import imutils
import cv2

REFOBJ_WIDTH = 1 #cm, CHECK IF U ARE MANAGING METRICS CORRECTLY, OR DO I HAVE TO PASS FROM FT TO IN OR SMTNG
color = (255,255,255) #white

#returns the middle of two points
def middle(point1, point2):
    return((point1[0] + point2[0]) * 0.5, (point1[1] + point2[1]) * 0.5)

#PARA OBTENER LOS CUATRO PUNTOS PUEDES USAR BOX = CV2.MINAREARECT(CNT), CV2.CV.BOXPOINTS(BOX)
#eso lo usas en findColor, despues de sacar los contornos, dentro del ciclo for

def clockwise_pts(pts):
    xSort = pts[np.argsort(pts[:,0]), :]
    leftMost = xSort[:2, :] #select the first two rows and all the colums of it.
    rightMost = xSort[2:, :]
    
    leftmost = leftMost[np.argsort(leftMost[:, 1]), :] #sort them by their Y coordinate
    tl, bl = leftmost
    #print(f"sorted tl: {tl} bl: {bl}")
    right = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0] #distances from tl to both points of rightmost
    br, tr = rightMost[np.argsort(right)[::-1], :]
    #print(f"sorted tr: {tr} br: {br}")

    return np.array([tl, tr, br, bl])

#not used yet
def measure_size(img, copy): 
    '''gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray_image, (7, 7), 0)
    #img preprocessing'''
    #edge detection
    edge = cv2.Canny(img, 50, 100)
    edge = cv2.dilate(edge, None, iterations = 1)
    edge = cv2.erode(edge, None, iterations = 1)
    cv2.imshow("edge", edge)

    #contouring
    fcontours, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    #useful for left reference objects, sorts them from left to right
    (fcontours, _) = contours.sort_contours(fcontours)
    ppm = None #pixels per metric

    #box analysis
    for cnt in fcontours:
        if cv2.contourArea(cnt) < 500:
            continue #jump to next iteration
        box = cv2.minAreaRect(cnt) #most accurate rectangle surrounding of a contour
        box_pts = cv2.boxPoints(box) #retrieve the coordinates of bounding boxes
        box_pts = np.array(box_pts, dtype="int")
        #print(box_pts)
        clockCoor = clockwise_pts(box_pts)#order coordinates in a clockwise manner for easier manipulation
        cv2.drawContours(copy, [clockCoor.astype("int")], -1, (255, 255, 0), 2) #draw clockwise coordinates based surrounding box
        #for (x, y) in cnt:
         #   cv2.circle(copy, (int(x), int(y)), 2, (255, 0, 0), 2)

#Returns a refObject, calculates distance between two objects if refObj already exist
def distance(imgCopy, refObj,objCoor,  objCenter): #objCoorX and Y are teh center of other object that is NOT the reference one, for example, the ball
    #these three lines are not used when refObj is None
    centerX = np.average(objCoor[:,0])
    centerY = np.average(objCoor[:,1])
    ballCenter = objCenter

    if refObj is None:
        #if NO refObj, creates one with the actual taken coordinates (while in a for loop)
        tl, tr, br, bl = objCoor
        centerLeftX, centerLeftY = middle(tl, bl)
        centerRightX, centerRightY = middle(tr, br)

        distHorizon = dist.euclidean((centerLeftX, centerLeftY), (centerRightX, centerRightY))
        #declare reference object with it's box, it's center and ppm.
        refObj = (objCoor, (centerX, centerY), distHorizon / REFOBJ_WIDTH)
    else:
        refBox, refCenter, ppm = refObj
    
        cv2.drawContours(imgCopy, [refBox.astype("int")], -1, (255,0,255), 2)
        cv2.drawContours(imgCopy, [objCoor.astype("int")], -1, (0,0,0), 2)

        #circles to create connections
        cv2.circle(imgCopy,(int(refCenter[0]),int(refCenter[1])), 5, color, -1)
        cv2.circle(imgCopy, (int(ballCenter[0]), int(ballCenter[1])), 5, color, -1)
        cv2.line(imgCopy, (int(refCenter[0]),int(refCenter[1])), (int(ballCenter[0]), int(ballCenter[1])), color, 2)

        #get the distance (in  pixels) from one pt to another
        distAB = dist.euclidean(refCenter, objCenter) #falta poner en ppm, antes lo dividias entre ppm del refObj
        midX, midY = middle(refCenter, objCenter)
        cv2.putText(imgCopy, "{:.1f}cm(IN)".format(distAB), (int(midX), int(midY) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)
    
    return refObj
    
        




















#**if run in terminal**
#To use these arguments, you just put of_arg["image"] or of_arg["width"]
'''argument = argparse.ArgumentParser()
argument.add_argument("-i", "--image", required = True, help = "Image path")
argument.add_argument("-w", "--width", required = True, help = "Reference object width", type = float)
#return a dictionary with the arguments
of_arg = vars(argument.parse_args()) #ofitial arguments'''

