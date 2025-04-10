
import cv2
import numpy as np
import imutils
from imutils import contours
from scipy.spatial import distance as dist
from size_measure import clockwise_pts, middle, distance
from homography import getHomography
import time
import socket
import struct

#USAR KD TREE PARA RUIDO!!!!!
#Aplica warp perspective para que se pongan solas las medidas

#in HSV 
colorParams = [0,103,31,20,255,255] #most accurate HSV values for test ball (bright orange) 0, 63, 255, 179, 255, 255
#checa la foto donde esta la terminal medio cubierta con los valores HSV que probaste con Alberto
#refColorParams = [0, 0, 0, 0, 0, 0]  # white 
refCenter = (320, 240) #in pixels
referenceWidth = 2.9  # Test width
ppm = None

realFieldCoors = [[0, 0], #tl
                  [68, 0], #tr
                  [68, 55], #br
                  [0, 55]] # bl

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

def mids(img, ppm, tl, tr, br, bl):
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

    distTop2Bott= dist.euclidean((int(topX), int(topY)), (int(bottomX), int(bottomY)))
    distLeft2Right = dist.euclidean((int(leftX), int(leftY)), (int(rightX), int(rightY))) 

    if ppm is None:
        avrgDiameterPixels = (distLeft2Right + distTop2Bott) / 2
        ppm = avrgDiameterPixels / referenceWidth

    #get real size of object
    szVert = distTop2Bott / ppm
    szHori = distLeft2Right / ppm
    
    #print(f"vert: {szVert}, hor: {szHori}")
    cv2.putText(img, f"{szVert:.1f}cm", (int(topX - 15), int(topY - 10)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(img, f"{szHori:.1f}cm", (int(rightX + 10), int(rightY)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)


def findContoursAndSize(img, copy):
    area = 0
    (cX, cY) = (0, 0)
    objCenter = 0
    #contours is a list of all shapes found in a frame
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    if not contours:
        return (0, 0)
        
    # Select biggest contour
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    cnt = contours[0] #first contour (biggest)
    
    area = cv2.contourArea(cnt)
    if area > 40:
        #Approximate the number of corners of the shape
        box = cv2.minAreaRect(cnt)
        box_pts = cv2.boxPoints(box)
        box_pts = np.array(box_pts, dtype="int")
        clockCoor = clockwise_pts(box_pts)
        cX = np.average(clockCoor[:, 0])
        cY = np.average(clockCoor[:, 1]) 
        #calculate the distance between the center and the Obj (ball in this case)
        #distance(copy, refCenter, clockCoor, (cX, cY), H) #draws distances from ball's and ref's centers
        cv2.drawContours(copy, [clockCoor.astype("int")], -1, (255, 255, 0), 2) #draw ball's bounding box
        for (x, y) in clockCoor: 
            cv2.circle(copy, (int(x), int(y)), 5, (0, 0, 255), -1) # circles in edges
        mids(copy, ppm, clockCoor[0], clockCoor[1], clockCoor[2], clockCoor[3]) #mid lines in shape, includes size
        #topLeft = tuple(clockCoor[0])
        #bottomRight = tuple(clockCoor[2])
    
    return (cX, cY)

#main function
def findObject(image, copy, ppm=None): 
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV", imgHSV)

    # Ball mask, already have refObj
    lower = np.array(colorParams[0:3])
    upper = np.array(colorParams[3:6])
    
    mask = cv2.inRange(imgHSV, lower, upper)
    gaussian = cv2.GaussianBlur(mask, (3,3), 0) #small kernel to mantain 
    
    # Calculate thresholds based on median of the image
    sigma = 0.33
    v = np.median(gaussian)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edges = cv2.Canny(gaussian, lower, upper)

    kernel = np.ones((3, 3), np.uint8)
    dilation = cv2.dilate(edges, kernel, iterations=1)

    cv2.imshow("mask", mask)
    cv2.imshow("gaussian", gaussian)
    cv2.imshow("edges", edges)
    cv2.imshow("dilate", dilation)
    
    objCenter = findContoursAndSize(dilation, copy)
    #if the center is detected, get it's coordinates in real field coordinates
    if objCenter:
        #give the correct format to the pt for use in perspectiveTransform
        objCenterPt = np.array([[objCenter[0], objCenter[1]]], dtype="float32")
        objCenterPt = np.array([objCenterPt])
        realFldCoors = cv2.perspectiveTransform(objCenterPt, H)[0][0]
        cv2.putText(copy, f"Real: ({realFldCoors[0]:.1f}, {realFldCoors[1]:.1f}) cm", (int(objCenter[0] + 50), int(objCenter[1] + 20)),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.imshow("mask", mask)
    #returns refObj(if object used) and ppm for those global variables to be in constant and correct changing state.
    return realFldCoors #regresar coordenadas REALES


cap = cv2.VideoCapture(2) #2 for external devices
cap.set(3, 640) #width
cap.set(4, 480) #height
cap.set(10, 20) #brightness

print("Homography Calibration. Click the four corners of the field in order TL, TR, BR, BL")
H = getHomography(cap, realFieldCoors)

#Declaring relay port and ip of the esp32  
RELAY_IP = "10.42.0.189"  # Replace with your esp
PORT_IP = 1234

while True:
    tpast = time.time()

    success, img = cap.read()
    img_copy = img.copy()
    if success:
        #image preprocessing
        #img_blur = cv2.GaussianBlur(img, (7, 7), 0)
        cv2.imshow("blur", img)
        #coordinates
        objCoors = findObject(img, img_copy, ppm)
        send_coordinates(objCoors[0], objCoors[1], RELAY_IP, PORT_IP)
        print(f"x: {objCoors[0]}, y: {objCoors[1]}")
        #objCenter are the coordinates we want for the robot to make decisions
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