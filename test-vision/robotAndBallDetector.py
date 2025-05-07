import cv2
import numpy as np
from size_measure import clockwise_pts, middle
from homography import getHomography
import time
import socket
import struct
from scipy.spatial import KDTree
from ultralytics import YOLO
from Model_use import bb_center_orien 

'''                        HC
HSV's -> Check for local circumstances in real competition
HSV's -> For robots 
IP's-> robotAndBallDetector.py ,  ballDetector.py
Homography -> Manually set homography '''

#fusion 
model = YOLO('/home/daniela/Desktop/VSSS/larc-vsss-2025/VSSS_modelM/runs/detect/custom-yolov8m/weights/best.pt')

RELAY_IP = "192.168.0.171" 
BALL = 1200 #for ball detections, IP for robot detections is in Model_use.py


#backup coordinates
ball_positions = []
MOVING_AVG_WINDOW = 5 #Tamaño de la ventana para la media movil
#in HSV Ball detection 
colorParams = [0, 144, 159, 15, 255, 255] #0, 203, 77, 9, 255, 228
#checa la foto donde esta la terminal medio cubierta con los valores HSV que probaste con Alberto
#0, 188, 197, 179, 255, 255

realFieldCoors = [[0, 0], #tl
                  [150, 0], #tr
                  [150, 130], #br
                  [0, 130]] # bl

CAMERA_HEIGHT = 200 #cm
clicked_points = []


#Communication python to esp32
def send_coordinates(x_coord, y_coord, relay_ip, relay_port):
    """
    Send two float coordinates to C++ relay via UDP
    Args:
        x_coord (float): X coordinate value
        y_coord (float): Y coordinate value
        relay_ip (str): IP address of the C++ relay
        relay_port (int): UDP port of the C++ relay
    """
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Pack the two float values into bytes
    # 'ff' format means two 32-bit float values
    data = struct.pack('ff', x_coord, y_coord) # Use 'e' for 16-bit floats 
    # Send the data
    sock.sendto(data, (relay_ip, relay_port))
    # Close the socket
    sock.close()

'''Removes noise, uses center of obj and pts near it to calculate a median of 
 the distances between them, and find the points that are in that valid area'''
def filter_noise(pts, center, radius_factor=1.2):
    if pts is None or len(pts) == 0:
        return None

    #radio del circulo
    distances = np.linalg.norm(pts - center, axis=1)
    median_radius = np.median(distances)

    #construir el kdTree
    kdTree = KDTree(pts)

    #Filtrar puntos válidos
    valid_index = kdTree.query_ball_point(center, radius_factor * median_radius)
    valid_index = np.array(valid_index)
    filtered_pts = pts[valid_index]

    return filtered_pts


#draw mid lines and referal points
def mids(img, tl, tr, br, bl):
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

#Returns center and general ball points
def findContoursAndSize(img, copy):
    area = 0
    (cX, cY) = (0, 0)
    objCenter = 0
    #contours is a list of all shapes found in a frame
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return (0, 0), None
    
    # Filtrar por área y circularidad
    valid_contours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)
        
        if area > 10 and 0.2 < circularity < 1.2:  # Ajusta estos valores
            valid_contours.append(cnt)

    if len(valid_contours) > 0:  
        # Select biggest contour
        contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)
        cnt = contours[0] #first contour (biggest)
        
        #puntos del contorno
        epsilon = 0.01 * cv2.arcLength(cnt,True) #tolerancia para la simplificación del contorno.
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        form_pts = approx.squeeze() #total ball pts to use in the KD-Tree

        #calculate an approximate center before calculating with real ones
        moments = cv2.moments(cnt)
        if moments["m00"] != 0:
            cX = moments["m10"] / moments["m00"]
            cY = moments["m01"] / moments["m00"]

            #get filtered points with initial center
            filterPts = filter_noise(form_pts, (cX, cY))

            #Approximate the number of corners of the shape
            box = cv2.minAreaRect(filterPts)
            box_pts = cv2.boxPoints(box)
            box_pts = np.array(box_pts, dtype="int")
            clockCoor = clockwise_pts(box_pts)
            cX = np.average(clockCoor[:, 0])
            cY = np.average(clockCoor[:, 1]) 
            
            return (cX, cY), form_pts
    return (0, 0), None

#returns the next position of the ball using polynomial extrapolation
#this is used to predict the next position of the ball
#in case the ball is not detected in the current frame
def polynomial_extrapolation(positions, degree=2):
    n = len(positions)
    if n < degree + 1:
        return positions[-1]  # Nqot enough points for a fit
    times = np.arange(n)
    poly_coeffs = np.polyfit(times, positions, degree)
    next_time = n
    next_position = np.polyval(poly_coeffs, next_time)
    return next_position

#main function. Returns object center with img preprocessing
def findObject(image, copy, H): 
    global ball_positions #allows the function to modify the global variable
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Ball mask, already have refObj
    lower = np.array(colorParams[0:3])
    upper = np.array(colorParams[3:6])
    mask = cv2.inRange(imgHSV, lower, upper)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    cv2.imshow("Better mask", mask)

    objCenter,objPts = findContoursAndSize(mask, copy)
    
    #if the center is detected, get it's coordinates in real field coordinates
    if objCenter:
        cv2.circle(copy, (int(objCenter[0]), int(objCenter[1])),2, (255,0,0), 5)
        #give the correct format to the pt for use in perspectiveTransform
        objCenterPt = np.array([objCenter[0], objCenter[1]], dtype="float32")
        print(f"Normal: {objCenterPt[0]}, {objCenterPt[1]}")
        

        # Agregar las coordenadas suavizadas a la lista de posiciones
        ball_positions.append((objCenterPt[0], objCenterPt[1]))
        if len(ball_positions) > MOVING_AVG_WINDOW:
            ball_positions.pop(0) #mantain window size

        #apply polynomial extrapolation to the last 5 points
        if len(ball_positions) >= MOVING_AVG_WINDOW:
            # Extrapolar la posición de la pelota
            ball_positions_np = np.array(ball_positions)
            x_extrapolated = polynomial_extrapolation(ball_positions_np[:, 0])
            y_extrapolated = polynomial_extrapolation(ball_positions_np[:, 1])
            objCenterPt = (x_extrapolated, y_extrapolated)
        cv2.circle(copy, (int(objCenterPt[0]), int(objCenterPt[1])), 2, (0, 255, 0), 5) #draw the polynomial extrapolation point
        print(f"Extrapolado: {objCenterPt[0]}, {objCenterPt[1]}")

        realFldCoors = cv2.perspectiveTransform(np.array([[[objCenterPt[0], objCenterPt[1]]]], dtype="float32"), H)[0][0]
        cv2.putText(copy, f"({realFldCoors[0]:.1f}, {realFldCoors[1]:.1f}) cm", (int(objCenter[0] + 50), int(objCenter[1] + 20)),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, (255, 255, 255), 2)

        return (realFldCoors[0], realFldCoors[1]) #regresar coordenadas REALES
    else:
        return (0, 0)


def main():       
    cap = cv2.VideoCapture(2)#2 for external devices, sometimes 0 idkw
    cap.set(3, 640) #width
    cap.set(4, 480) #height

    print("Homography " \
    "Calibration. Click the four corners of the field in order TL, TR, BR, BL")
    H = getHomography(cap, realFieldCoors)

    window_name = "Detección"
    #cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Bucle principal
    while True:
        tpast = time.time()
        
        success, img = cap.read()

        if success:
            # Detección de robots con el modelo YOLO
            img_copy = img.copy()  # Copia del frame para detección de la pelota
            results = model.track(img)

            # Detección de pelota
            objCoorsCenter = findObject(img, img_copy, H)  # Coordenadas reales de la pelota
            if objCoorsCenter[0] != 0 and objCoorsCenter[1] != 0:
                send_coordinates(objCoorsCenter[0], objCoorsCenter[1], RELAY_IP, BALL)

                print(f"x: {objCoorsCenter[0]}, y: {objCoorsCenter[1]}")
            else:
                send_coordinates(0, 0, RELAY_IP, BALL)

            #Detección de robots
            if results:
                detect_img = results[0].plot()
                robots = bb_center_orien(results, img_copy, H)  # Procesar orientación y centro de los robots
                for robot_id, robot in robots.items():
                    # Enviar coordenadas de los robots
                    robot.send_data(RELAY_IP)
                cv2.imshow("Model", detect_img)
                cv2.imshow("Detections", img_copy)
            if cv2.waitKey(1) == ord('q'):
                break
        else:
            print("No camera")
        # Control de salida 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            #print(f"FPS: {fps:.2f}")
            break
        
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()