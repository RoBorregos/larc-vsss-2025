from scipy.spatial import distance as dist #multidimensional operations provider
from imutils import perspective ####library for computer vision, perspective can order the points in tl, tr, br, bl
from imutils import contours
import numpy as np
import argparse #for console arguments **Not necessary**
import imutils
import cv2

REFOBJ_WIDTH = 25 #cm, CHECK IF U ARE MANAGING METRICS CORRECTLY, OR DO I HAVE TO PASS FROM FT TO IN OR SMTNG
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



#calculates distance between the center of the image and the object
def distance(imgCopy, refCenter,objCoor,  objCenter, homography): #-----------------------------NOT NECESSARY---------------------------------------

    ballCenter = objCenter
    #adapt format
    adaptObj = np.array([[[ballCenter[0], ballCenter[1]]]], dtype="float32")
    ballRlCoors = cv2.perspectiveTransform(adaptObj, homography)[0][0]

    refAdapt = np.array([[[refCenter[0], refCenter[1]]]], dtype="float32")
    refRlCoors = cv2.perspectiveTransform(refAdapt, homography)[0][0]

    cv2.drawContours(imgCopy, [objCoor.astype("int")], -1, (0,0,0), 2)

    #circles to create connections
    cv2.circle(imgCopy,(int(refCenter[0]),int(refCenter[1])), 5, color, -1)
    cv2.circle(imgCopy, (int(ballCenter[0]), int(ballCenter[1])), 5, color, -1)
    cv2.line(imgCopy, (int(refCenter[0]),int(refCenter[1])), (int(ballCenter[0]), int(ballCenter[1])), color, 2)

    #get the distance (in  pixels) from one pt to another
    distAB = dist.euclidean(refRlCoors, ballRlCoors) #falta poner en ppm, antes lo dividias entre ppm del refObj
    midX, midY = middle(refCenter, objCenter)
    cv2.putText(imgCopy, "{:.1f}cm".format(distAB), (int(midX), int(midY) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2) #----------------------------

