
import cv2
import numpy as np
from size_measure import clockwise_pts, middle
from homography import getHomography
import time
import socket
import struct
from scipy.spatial import KDTree


#in HSV 
colorParams = [0, 203, 77, 9, 255, 228] #most accurate HSV values for test ball (bright orange) 0, 63, 255, 179, 255, 255
#checa la foto donde esta la terminal medio cubierta con los valores HSV que probaste con Alberto
#refColorParams = [0, 0, 0, 0, 0, 0]  # white 
refCenter = (320, 240) #in pixels
referenceWidth = 2.9  # Test width
#changess
realFieldCoors = [[0, 0], #tl
                  [150, 0], #tr
                  [150, 130], #br
                  [0, 130]] # bl

cameraCoors = [[0,0], 
               [640, 0], 
               [640, 480],
               [0, 480]]

CAMERA_HEIGHT = 200 #cm
clicked_points = []

#Communication python to esp32
def send_coordinates(x_coord, y_coord, relay_ip, relay_port=1234):
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
            '''cv2.drawContours(copy, [clockCoor.astype("int")], -1, (255, 255, 0), 2) #draw ball's bounding box
            for (x, y) in clockCoor: 
                cv2.circle(copy, (int(x), int(y)), 5, (0, 0, 255), -1) # circles in edges
            mids(copy, clockCoor[0], clockCoor[1], clockCoor[2], clockCoor[3]) #mid lines in shape
            topLeft = tuple(clockCoor[0])
            bottomRight = tuple(clockCoor[2])'''
            
            return (cX, cY), form_pts
    return (0, 0), None

#main function. Returns object center with img preprocessing
def findObject(image, copy): 
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Ball mask, already have refObj
    lower = np.array(colorParams[0:3])
    upper = np.array(colorParams[3:6])
    mask = cv2.inRange(imgHSV, lower, upper)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    cv2.imshow("Better mask", mask)

    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
    #mask = cv2.dilate(mask, kernel_dilate, iterations=2)

    objCenter,objPts = findContoursAndSize(mask, copy)
    
    #if the center is detected, get it's coordinates in real field coordinates
    if objCenter:
        cv2.circle(copy, (int(objCenter[0]), int(objCenter[1])),2, (255,0,0), 5)
        #give the correct format to the pt for use in perspectiveTransform
        objCenterPt = np.array([[objCenter[0], objCenter[1]]], dtype="float32")
        objCenterPt = np.array([objCenterPt])
        realFldCoors = cv2.perspectiveTransform(objCenterPt, H)[0][0]
        cv2.putText(copy, f"({realFldCoors[0]:.1f}, {realFldCoors[1]:.1f}) cm", (int(objCenter[0] + 50), int(objCenter[1] + 20)),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, (255, 255, 255), 2)
    if objCenter != None:
        return (realFldCoors[0], realFldCoors[1]) #regresar coordenadas REALES
    else:
        return (0, 0)
        
cap = cv2.VideoCapture(2) #2 for external devices
cap.set(3, 640) #width
cap.set(4, 480) #height

print("Homography Calibration. Click the four corners of the field in order TL, TR, BR, BL")
H = getHomography(cap, realFieldCoors)
#save homography matrix for further use in robot localization

#Declaring relay port and ip of the esp32  
RELAY_IP = "10.42.0.45"  # Replace with your esp
PORT_IP = 1234

while True:
    tpast = time.time()

    success, img = cap.read()
    if success:
        img_copy = img.copy()
        #coordinates of the center 
        objCoorsCenter = findObject(img, img_copy)

        #Kalman filter usage for coordinate prediction
        if objCoorsCenter != (0,0):
            #use UDP sending coordinates algorithm
            send_coordinates(objCoorsCenter[0], objCoorsCenter[1], RELAY_IP, PORT_IP)
            print(f"x: {objCoorsCenter[0]}, y: {objCoorsCenter[1]}")
        
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