from ultralytics import YOLO
import cv2
import time
import math
import numpy as np
import json #if communication tells you he needs the info in json format
from homography import getHomography
import struct
import socket


#changes
model = YOLO('/home/daniela/Desktop/VSSS/larc-vsss-2025/VSSSModel/runs/detect/custom_VSSS_model/weights/best.pt')

cap = cv2.VideoCapture(2)
cap.set(3, 640) #width
cap.set(4, 480) #height

realFieldCoors = [[0, 0], #tl
                  [150, 0], #tr
                  [150, 130], #br
                  [0, 130]] # bl

hsvRanges = {
    'blue' : {'lower':[95, 122, 80], 'upper': [111, 255, 255]}, #h_min =  95  h_max =  111  Sat_min =  122  Sat_max =  255  Val_min =  80  Val_max =  255
    'yellow' : {'lower': [4, 19, 51], 'upper':[55, 196, 255] } #h_min =  4  h_max =  55  Sat_min =  19  Sat_max =  196  Val_min =  51  Val_max =  255
}

#Communication python to esp32
def send_coordinates(x, y, orientation, relay_ip, relay_port):
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
    data = struct.pack('fff', x, y, orientation) # Use 'e' for 16-bit floats 
    # Send the data
    sock.sendto(data, (relay_ip, relay_port))
    # Close the socket
    sock.close()




def transform_coors(pt, H):
    pt = np.array([[pt]], type="float32")
    transformed_pt = cv2.perspectiveTransform(pt, H)
    return transformed_pt[0][0]


def get_color_centroid(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    max_area = 0
    centroid = None
    biggest_color = None
    
    for color, ranges in hsvRanges.items():
        lower = np.array(ranges['lower'])
        upper = np.array(ranges['upper'])
        mask = cv2.inRange(hsv_img, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            #returns the biggest contour based on it's area
            largest_cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_cnt)
            if area > max_area:
                #cv2.imshow("Mask", mask)
                max_area = area
                M = cv2.moments(largest_cnt)
                if M['m00'] > 0:
                    cX = int(M["m10"] / M['m00']) #X entre area
                    cY = int(M['m01'] / M['m00']) #Y entre area
                    centroid = (cX, cY)
                    biggest_color = color
                    print(f"Centro valido: {centroid}")
    
    #if not contours found, return None
    return centroid, biggest_color

def get_orientation(img, color_centroid):
    bb_shape = img.shape
    #the img here is only used for debug (to see the orientation line)
    if color_centroid != None:
        cX = float(bb_shape[0] / 2)
        cY = float(bb_shape[1] / 2)
        robot_centroid = (cX, cY)
        print(f"SHAPE: {img.shape}")

        dx = robot_centroid[0] - color_centroid[0]
        dy = robot_centroid[1] - color_centroid[1]
        print(f"Robot: {robot_centroid}\n Color: {color_centroid}")
        #return orientation in degrees; change depending on control needs
        orientation = math.degrees(math.atan2(dy, dx)) % 360
        cv2.line(img, (int(color_centroid[0]), int(color_centroid[1])), (int(robot_centroid[0]), int(robot_centroid[1])), (0,255, 0), 2)
        return orientation
    else:
        return None


#returns list of (x, y) tuples for the center of each detected bb (bounding box)
def bb_center_orien(results, img, H):
    robots_data = {"robots" : []}
    robot_orien = 0
    robot_coors = (0.0, 0.0)
    team = 0

    for res in results:
            boxes = res.boxes #variable with all the bb's detected in the frame
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]

                #get robot's pattern
                class_id = int(box.cls[0]) + 1 #detected class index

                #get robot's center (position regarding general view, not roi)
                x_center = float((x1 + x2) / 2)
                y_center = float((y1 + y2) / 2)

                #robot's orientation based on largest area color
                roi = img[int(y1):int(y2), int(x1):int(x2)]
                
                
                color_centroid, _ = get_color_centroid(roi)
                orien = get_orientation(roi, color_centroid)
                if orien != None:
                    orien = round(float(orien), 4)
                print(f"Robot {class_id}, orientation: {orien}")

                #get the real center coordinates
                robot_coors = np.array([[x_center, y_center]], dtype="float32")
                robot_coors = np.array([robot_coors])
                real_robot_coors = cv2.perspectiveTransform(robot_coors, H)[0][0]
                
                send_coordinates(real_robot_coors[0], real_robot_coors[1], orien,  RELAY_IP, 1235)

                #make them int for cv2.circle
                x_center = int(x_center)
                y_center = int(y_center) 
                #debug
                cv2.circle(img, (x_center, y_center), 2, (0,0,255), -1)
                #cv2.putText(res_img, f"({float(real_robot_coors[0]):.1f}, {float(real_robot_coors):.1f})", 
                        #((x_center), (y_center)), 
                        #cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

H = getHomography(cap, realFieldCoors)

RELAY_IP = "192.168.0.161"  # Replace with your esp
#PORT_IP = 1234 #change port for robot data communication

#main loop
while True:
    tpast = time.time()
    success, img = cap.read()

    if success:
        results = model.predict(img)
        if results:
            res_img = results[0].plot()
            #center and orientation for each bb detected
            bb_center_orien(results, res_img, H)
            cv2.imshow("Model prediction", res_img)
        else:
            print("No se usa modelo")
    else:
        print("No success")
    tnow = time.time()
    totalTime = tnow - tpast
    fps = 1 / totalTime
    if cv2.waitKey(1) == ord('q'):
        print(f"fps: {fps}")
        break