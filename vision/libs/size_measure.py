from scipy.spatial import distance as dist 
from imutils import perspective 
from imutils import contours
import numpy as np
import cv2

#white
color = (255,255,255) 

def middle(point1, point2):
    """returns the middle of two points"""
    return((point1[0] + point2[0]) * 0.5, (point1[1] + point2[1]) * 0.5)

def clockwise_pts(pts):
    xSort = pts[np.argsort(pts[:,0]), :]
    #select the first two rows and all the colums of it.
    leftMost = xSort[:2, :] 
    rightMost = xSort[2:, :]
    
    #sort them by their Y coordinate
    leftmost = leftMost[np.argsort(leftMost[:, 1]), :] 
    tl, bl = leftmost
    #distances from tl to both points of rightmost
    right = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0] 
    br, tr = rightMost[np.argsort(right)[::-1], :]

    return np.array([tl, tr, br, bl])

def distance(imgCopy, refCenter,objCoor,  objCenter, homography): 
    '''calculates distance between the center of the image and the object'''
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
    distAB = dist.euclidean(refRlCoors, ballRlCoors) 
    midX, midY = middle(refCenter, objCenter)
    cv2.putText(imgCopy, "{:.1f}cm".format(distAB), (int(midX), int(midY) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)

