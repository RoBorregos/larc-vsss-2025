import cv2
import numpy as np
import imutils
from imutils import contours
from scipy.spatial import distance as dist

cap = cv2.VideoCapture(0)
cap.set(3, 640)  # width
cap.set(4, 480)  # height
cap.set(10, 20)  # brightness

# Dimensiones reales de la cancha
field_width_cm = 300  # Ajusta según las medidas reales de tu cancha
field_height_cm = 200  # Ajusta según las medidas reales de tu cancha

# Parámetros iniciales (ajusta según el color del borde del campo)
field_color_lower = np.array([0, 0, 200])  # Blanco o color del borde (ajusta con trackbars)
field_color_upper = np.array([179, 50, 255])  # Ajusta según el color

# In HSV
colorParams = [0, 63, 255, 179, 255, 255]  # Bola naranja (ajusta si es necesario)
referenceWidth = 2.9  # Diámetro real de la bola en cm

ppm = None  # Inicializar como variable global
ball_coords = (0, 0)  # Coordenadas de la pelota en la imagen
field_center = (field_width_cm / 2, field_height_cm / 2)  # Centro del campo en cm
H = None  # Matriz de homografía (se calculará dinámicamente)

def normalize_coordinates(coords):
    cX, cY = coords
    normalized_x = (cX - 320) / 320  # -1 a 1 (opcional, para el robot)
    normalized_y = (cY - 240) / 240  # -1 a 1
    return (normalized_x, normalized_y)

def send_coordinates_to_robot(coords):
    normalized_coords = normalize_coordinates(coords)
    print(f"Enviando coordenadas normalizadas al robot: {normalized_coords}")
    # Implementa la lógica de envío aquí (por ejemplo, serial o socket)

def transform_point(point, H):
    """Transforma un punto de la imagen al sistema de coordenadas del campo."""
    if H is None:
        return point  # Si no hay homografía, devolver coordenadas originales
    point_homog = np.array([point[0], point[1], 1], dtype=np.float32)
    transformed_homog = H @ point_homog
    transformed = transformed_homog / transformed_homog[2]  # Normalizar
    return (transformed[0], transformed[1])

def get_field_contour(img):
    """Detecta el contorno del campo y devuelve sus esquinas."""
    global H
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(imgHSV, field_color_lower, field_color_upper)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None

    # Tomar el contorno más grande (asumiendo que es el campo)
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 10000:  # Umbral para evitar falsos positivos
        return None, None

    # Aproximar a un rectángulo
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    if len(approx) == 4:  # Si es un rectángulo
        approx = approx.reshape(4, 2)
        # Ordenar esquinas (arriba-izquierda, arriba-derecha, abajo-derecha, abajo-izquierda)
        rect = np.zeros((4, 2), dtype=np.float32)
        s = approx.sum(axis=1)
        rect[0] = approx[np.argmin(s)]  # Arriba-izquierda
        rect[2] = approx[np.argmax(s)]  # Abajo-derecha
        diff = np.diff(approx, axis=1)
        rect[1] = approx[np.argmin(diff)]  # Arriba-derecha
        rect[3] = approx[np.argmax(diff)]  # Abajo-izquierda

        # Definir puntos del campo real (esquinas en cm)
        field_points = np.array([
            [0, 0],
            [field_width_cm, 0],
            [field_width_cm, field_height_cm],
            [0, field_height_cm]
        ], dtype=np.float32)

        # Calcular homografía
        H, _ = cv2.findHomography(rect, field_points)
        return rect, H
    return None, None

def mids(img, ppm, tl, tr, br, bl):  # Calcular ppm y dimensiones
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
        ppm = avrgDiameterPixels / referenceWidth  # Calibración inicial

    szVert = distTop2Bott / ppm
    szHori = distLeft2Right / ppm
    
    print(f"vert: {szVert:.2f}cm, hor: {szHori:.2f}cm")
    cv2.putText(img, f"{szVert:.1f}cm", (int(topX - 15), int(topY - 10)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(img, f"{szHori:.1f}cm", (int(rightX + 10), int(rightY)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)
    return ppm

def findContoursAndSize(img, copy, ppm=None):
    global ball_coords, H
    area, peri, radius = 0, 0, 0
    topLeft, bottomRight = (0, 0), (0, 0)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    if not contours:
        return 0, 0, 0, (0, 0), (0, 0), ppm
    
    #contours = imutils.grab_contours(contours)
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
        cX = int(np.average(clockCoor[:, 0]))
        cY = int(np.average(clockCoor[:, 1]))
        ball_coords = (cX, cY)

        # Transformar las coordenadas de la pelota al sistema del campo
        ball_field_coords = transform_point((cX, cY), H)
        
        # Calcular distancia desde el centro del campo
        if ball_field_coords is not None and H is not None:
            dist_to_center = dist.euclidean(field_center, ball_field_coords)
            cv2.circle(copy, (cX, cY), 5, (0, 255, 0), -1)  # Centro de la pelota
            cv2.circle(copy, (320, 240), 5, (255, 0, 255), -1)  # Centro de la imagen
            cv2.line(copy, (320, 240), (cX, cY), (255, 255, 255), 2)
            cv2.putText(copy, f"Dist: {dist_to_center:.1f}cm", (cX, cY - 20),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(copy, f"Field: ({ball_field_coords[0]:.1f}, {ball_field_coords[1]:.1f})cm",
                        (cX, cY - 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)

        cv2.drawContours(copy, [clockCoor.astype("int")], -1, (255, 255, 0), 2)
        for (x, y) in clockCoor:
            cv2.circle(copy, (int(x), int(y)), 5, (0, 0, 255), -1)
        ppm = mids(copy, ppm, clockCoor[0], clockCoor[1], clockCoor[2], clockCoor[3])
        topLeft = tuple(clockCoor[0])
        bottomRight = tuple(clockCoor[2])
        radius = np.sqrt(area / np.pi)

    return radius, area, peri, topLeft, bottomRight, ppm

def findColor(image, copy, ppm=None):
    global H
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Detección del campo para homografía
    field_rect, new_H = get_field_contour(image)
    if field_rect is not None and new_H is not None:
        H = new_H
        cv2.polylines(copy, [field_rect.astype(np.int32)], True, (0, 255, 255), 2)

    # Máscara para la bola
    lower = np.array(colorParams[0:3])
    upper = np.array(colorParams[3:6])
    mask = cv2.inRange(imgHSV, lower, upper)
    
    # Preprocesamiento para reducir ruido
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)
    
    r, a, p, tl, br, ppm = findContoursAndSize(mask, copy, ppm)
    
    if a > 500:
        cv2.rectangle(copy, tl, br, (0, 255, 0), 5)
        cv2.putText(copy, "Ball", (br[0], br[1]), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)
        cv2.putText(copy, f"Img: ({ball_coords[0]}, {ball_coords[1]})", (ball_coords[0], ball_coords[1] - 40),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)
    if a > 100:
        print(f"Area: {a} Perimeter: {p} Radius: {r} Coords: {ball_coords}")
    
    cv2.imshow("mask", mask)
    return ppm

while True:
    success, img = cap.read()
    img_copy = img.copy()
    if success:
        img_blur = cv2.GaussianBlur(img, (5, 5), 0)
        ppm = findColor(img_blur, img_copy)
        send_coordinates_to_robot(ball_coords)
        cv2.imshow("Test", img_copy)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

def middle(point1, point2):  # Función auxiliar
    return ((point1[0] + point2[0]) * 0.5, (point1[1] + point2[1]) * 0.5)

def clockwise_pts(pts):  # Función auxiliar
    xSort = pts[np.argsort(pts[:, 0]), :]
    leftMost = xSort[:2, :]
    rightMost = xSort[2:, :]
    leftmost = leftMost[np.argsort(leftMost[:, 1]), :]
    tl, bl = leftmost
    right = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    br, tr = rightMost[np.argsort(right)[::-1], :]
    return np.array([tl, tr, br, bl], dtype=np.float32)