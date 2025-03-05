import cv2
import numpy as np
import imutils
from size_measure import clockwise_pts

cap = cv2.VideoCapture(0)
cap.set(3, 640)  # width
cap.set(4, 480)  # height
cap.set(10, 20)  # brightness

# Ajusta el rango HSV para el naranja de la bola
colorParams = [5, 100, 100, 25, 255, 255]  # [H_min, S_min, V_min, H_max, S_max, V_max]

def findContours(img, copy):
    area, peri, radius = 0, 0, 0
    topLeft, bottomRight = (0, 0), (0, 0)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    if not contours:
        return 0, 0, 0, (0, 0), (0, 0)
    
    contours = imutils.grab_contours(contours)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            cv2.drawContours(copy, [cnt], -1, (0, 255, 0), 2)
            peri = cv2.arcLength(cnt, True)
            box = cv2.minAreaRect(cnt)
            box_pts = cv2.boxPoints(box)
            box_pts = np.array(box_pts, dtype="int")
            clockCoor = clockwise_pts(box_pts)
            cv2.drawContours(copy, [clockCoor.astype("int")], -1, (255, 255, 0), 2)
            topLeft = tuple(clockCoor[0])  # Top-left
            bottomRight = tuple(clockCoor[2])  # Bottom-right
            radius = np.sqrt(area / np.pi)
            break
    return radius, area, peri, topLeft, bottomRight

def findColor(image, copy):
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array(colorParams[0:3])
    upper = np.array(colorParams[3:6])
    mask = cv2.inRange(imgHSV, lower, upper)
    r, a, p, top_left, bottom_right = findContours(mask, copy)
    if a > 500:  # Solo muestra si es significativo
        cv2.rectangle(copy, top_left, bottom_right, (0, 255, 0), 5)
        cv2.putText(copy, "Ball", (top_left[0] + 10, top_left[1] - 10), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)
        print(f"Area: {a} Perimeter: {p} Radius: {r} TopLeft: {top_left} BottomRight: {bottom_right}")
    cv2.imshow("mask", mask)

while True:
    success, img = cap.read()
    img_copy = img.copy()
    if success:
        # Preprocesamiento
        img_blur = cv2.GaussianBlur(img, (7, 7), 0)  # Aumenta el kernel para más suavizado
        
        findColor(img_blur, img_copy)
        cv2.imshow("Test", img_copy)
        cv2.imshow("Blurred", img_blur)  # Para depuración
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()