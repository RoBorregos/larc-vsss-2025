from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import imutils
import cv2

REFOBJ_WIDTH = 1  # Ajusta según el tamaño real del objeto de referencia (en cm)
color = (255, 255, 255)  # Blanco

def middle(point1, point2):
    return ((point1[0] + point2[0]) * 0.5, (point1[1] + point2[1]) * 0.5)

def clockwise_pts(pts):
    xSort = pts[np.argsort(pts[:, 0]), :]
    leftMost = xSort[:2, :]
    rightMost = xSort[2:, :]

    leftmost = leftMost[np.argsort(leftMost[:, 1]), :]
    tl, bl = leftmost
    right = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    br, tr = rightMost[np.argsort(right)[::-1], :]

    return np.array([tl, tr, br, bl])

def measure_size(img, copy):
    edge = cv2.Canny(img, 50, 100)
    edge = cv2.dilate(edge, None, iterations=1)
    edge = cv2.erode(edge, None, iterations=1)
    cv2.imshow("edge", edge)

    fcontours, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    (fcontours, _) = contours.sort_contours(fcontours)
    ppm = None

    for cnt in fcontours:
        if cv2.contourArea(cnt) < 500:
            continue
        box = cv2.minAreaRect(cnt)
        box_pts = cv2.boxPoints(box)
        box_pts = np.array(box_pts, dtype="int")
        clockCoor = clockwise_pts(box_pts)
        cv2.drawContours(copy, [clockCoor.astype("int")], -1, (255, 255, 0), 2)

def distance(imgCopy, refObj, objCoor, objCenter):
    centerX = np.average(objCoor[:, 0])
    centerY = np.average(objCoor[:, 1])
    ball_center = objCenter

    if refObj is None:
        tl, tr, br, bl = objCoor
        centerLeftX, centerLeftY = middle(tl, bl)
        centerRightX, centerRightY = middle(tr, br)
        distHorizon = dist.euclidean((centerLeftX, centerLeftY), (centerRightX, centerRightY))
        refObj = (objCoor, (centerX, centerY), distHorizon / REFOBJ_WIDTH)
    else:
        ref_box, ref_center, ppm = refObj
        # Dibujar contorno del objeto de referencia
        cv2.drawContours(imgCopy, [ref_box.astype("int")], -1, (255, 0, 255), 2)
        # Dibujar contorno de la bola
        cv2.drawContours(imgCopy, [objCoor.astype("int")], -1, (0, 0, 0), 2)

        # Conectar centros
        cv2.circle(imgCopy, (int(ref_center[0]), int(ref_center[1])), 5, color, -1)
        cv2.circle(imgCopy, (int(ball_center[0]), int(ball_center[1])), 5, color, -1)
        cv2.line(imgCopy, (int(ref_center[0]), int(ref_center[1])), 
                 (int(ball_center[0]), int(ball_center[1])), color, 2)

        # Calcular y mostrar la distancia
        distAB = dist.euclidean(ref_center, ball_center) / ppm
        midX, midY = middle(ref_center, ball_center)
        cv2.putText(imgCopy, f"{distAB:.1f}cm", (int(midX), int(midY) - 10), 
                    cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 255, 255), 2)

    return refObj