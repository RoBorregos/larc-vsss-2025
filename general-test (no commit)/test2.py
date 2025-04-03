import cv2
import numpy as np
import imutils
from imutils import contours
from scipy.spatial import distance as dist
from test1 import clockwise_pts, middle, distance

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
cap.set(10, 20)

colorParams = [0, 63, 255, 179, 255, 255]  # Bola naranja 
refColorParams = [0, 0, 0, 0, 0, 0]  # Portería blanca (ajusta según el color real)
referenceWidth = 2.9  # Ajusta según el tamaño real de la bola

def mids(img, ppm, tl, tr, br, bl):
    box = (tl, tr, br, bl)
    (topX, topY) = middle(tl, tr)
    (bottomX, bottomY) = middle(bl, br)
    (leftX, leftY) = middle(tl, bl)
    (rightX, rightY) = middle(tr, br)

    cv2.circle(img, (int(topX), int(topY)), 5, (0, 0, 255), -1)
    cv2.circle(img, (int(bottomX), int(bottomY)), 5, (0, 0, 255), -1)
    cv2.circle(img, (int(rightX), int(rightY)), 5, (0, 0, 255), -1)
    cv2.circle(img, (int(leftX), int(leftY)), 5, (0, 0, 255), -1)

    cv2.line(img, (int(topX), int(topY)), (int(bottomX), int(bottomY)), (255, 255, 255), 2)
    cv2.line(img, (int(leftX), int(leftY)), (int(rightX), int(rightY)), (255, 255, 255), 2)

    distTop2Bott = dist.euclidean((int(topX), int(topY)), (int(bottomX), int(bottomY)))
    distLeft2Right = dist.euclidean((int(leftX), int(leftY)), (int(rightX), int(rightY)))

    if ppm is None:
        avrgDiameterPixels = (distLeft2Right + distTop2Bott) / 2
        ppm = avrgDiameterPixels / referenceWidth

    szVert = distTop2Bott / ppm
    szHori = distLeft2Right / ppm
    
    print(f"vert: {szVert:.2f}, hor: {szHori:.2f}")
    cv2.putText(img, f"{szVert:.1f}cm", (int(topX - 15), int(topY - 10)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(img, f"{szHori:.1f}cm", (int(rightX + 10), int(rightY)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)

def findContoursAndSize(img, copy, referenceObj=None, ppm=None):
    area, peri, radius = 0, 0, 0
    topLeft, bottomRight = (0, 0), (0, 0)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    if not contours:
        return 0, 0, 0, (0, 0), (0, 0), referenceObj, ppm
    
    #contours = imutils.grab_contours(contours)
    # Seleccionar solo el contorno más grande (la bola)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    cnt = contours[0]  # Tomar el contorno más grande
    
    area = cv2.contourArea(cnt)
    if area > 500:
        cv2.drawContours(copy, cnt, -1, (0, 255, 0), 2)
        peri = cv2.arcLength(cnt, True)
        box = cv2.minAreaRect(cnt)
        box_pts = cv2.boxPoints(box)
        box_pts = np.array(box_pts, dtype="int")
        clockCoor = clockwise_pts(box_pts)
        cX = np.average(clockCoor[:, 0])
        cY = np.average(clockCoor[:, 1])
        referenceObj = distance(copy, referenceObj, clockCoor, (cX, cY))
        cv2.drawContours(copy, [clockCoor.astype("int")], -1, (255, 255, 0), 2)
        for (x, y) in clockCoor:
            cv2.circle(copy, (int(x), int(y)), 5, (0, 0, 255), -1)
        mids(copy, ppm, clockCoor[0], clockCoor[1], clockCoor[2], clockCoor[3])
        topLeft = tuple(clockCoor[0])
        bottomRight = tuple(clockCoor[2])
        radius = np.sqrt(area / np.pi)
    
    return radius, area, peri, topLeft, bottomRight, referenceObj, ppm

def findColor(image, copy, referenceObj=None, ppm=None):
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Máscara para el objeto de referencia (portería)
    lower_ref = np.array(refColorParams[0:3])
    upper_ref = np.array(refColorParams[3:6])
    mask_ref = cv2.inRange(imgHSV, lower_ref, upper_ref)
    
    # Detectar el objeto de referencia
    contours_ref, _ = cv2.findContours(mask_ref, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
                referenceObj = distance(copy, None, clockCoor_ref, (cX_ref, cY_ref))
                cv2.drawContours(copy, [clockCoor_ref.astype("int")], -1, (255, 0, 255), 2)
                cv2.circle(copy, (int(cX_ref), int(cY_ref)), 5, (0, 255, 255), -1)
                break
    
    # Máscara para la bola
    lower = np.array(colorParams[0:3])
    upper = np.array(colorParams[3:6])
    mask = cv2.inRange(imgHSV, lower, upper)
    
    r, a, p, top_left, bottom_right, referenceObj, ppm = findContoursAndSize(mask, copy, referenceObj, ppm)
    if a > 500:
        cv2.rectangle(copy, top_left, bottom_right, (0, 255, 0), 5)
        cv2.putText(copy, "Ball", (bottom_right[0], bottom_right[1]), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)
    if a > 100:
        print(f"Area: {a} Perimeter: {p} Radius: {r}")
    cv2.imshow("mask", mask)
    cv2.imshow("mask_ref", mask_ref)
    return referenceObj, ppm

referenceObj = None
ppm = None
while True:
    success, img = cap.read()
    img_copy = img.copy()
    if success:
        img_blur = cv2.GaussianBlur(img, (5, 5), 0)
        referenceObj, ppm = findColor(img_blur, img_copy, referenceObj, ppm)
        cv2.imshow("Test", img_copy)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()