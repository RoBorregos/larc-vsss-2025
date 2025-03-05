from scipy.spatial import distance as dist #multidimensional operations provider
from imutils import perspective ####library for computer vision, perspective can order the points in tl, tr, br, bl
from imutils import contours
import numpy as np
import argparse #for console arguments **Not necessary**
import imutils
import cv2

#returns the middle between two points
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
    #fcontours = imutils.grab_contours(fcontours) #manages contours in an easier way (doesn't matter opencv version)

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


#test loop


        




















#**if run in terminal**
#To use these arguments, you just put of_arg["image"] or of_arg["width"]
'''argument = argparse.ArgumentParser()
argument.add_argument("-i", "--image", required = True, help = "Image path")
argument.add_argument("-w", "--width", required = True, help = "Reference object width", type = float)
#return a dictionary with the arguments
of_arg = vars(argument.parse_args()) #ofitial arguments'''

