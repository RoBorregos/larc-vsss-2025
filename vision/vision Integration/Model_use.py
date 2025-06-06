from ultralytics import YOLO
import torch
import cv2
import time
import math
import numpy as np
from vision.libs.homography import getHomography, warpChange, autoGetHomography
import struct
import socket
from collections import deque


class RobotData:
    def __init__(self, x=0.0, y=0.0, orientation=0.0, port = 1200, patterns = []):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.port = port
        self.centroid_history = deque(maxlen=5) #a centroid history for each robot
        self.orientation_history = deque(maxlen=5)
        self.patterns = patterns  
    def update(self, x, y, orientation):
        """
        Updates the robot's position and orientation.
        
        Args:
            x (float): New X-coordinate.
            y (float): New Y-coordinate.
            orientation (float): New orientation in radians.
        """
        self.x = x
        self.y = y
        self.orientation = orientation

    def moving_average_centroid(self, centroid):
        """
        Calculates the moving average of centroids for smoothing.
        
        Args:
            centroid (tuple): Current centroid (x, y).
        
        Returns:
            tuple: Smoothed centroid (x, y).
        """
        if centroid is not None:
            self.centroid_history.append(centroid)
            if len(self.centroid_history) > 0:
                avg_x = sum([c[0] for c in self.centroid_history]) / len(self.centroid_history)
                avg_y = sum([c[1] for c in self.centroid_history]) / len(self.centroid_history)
                return (avg_x, avg_y)
        return centroid

    def moving_average_orientation(self, new_orientation=None):
        """
        Calculates the moving average of orientations for smoothing.
        
        Args:
            new_orientation (float): Current orientation in radians.
        
        Returns:
            float: Smoothed orientation in radians.
        """
        if new_orientation is not None:
            self.orientation_history.append(new_orientation)
        if len(self.orientation_history) > 0:
            return sum(self.orientation_history) / len(self.orientation_history)
        return self.orientation
    
    def __str__(self):
        return f"x={self.x}, y={self.y}, orientation={self.orientation}, port={self.port}"
    
    def send_data(self, relay_ip):
        """
        Sends the robot's data (x, y, orientation) to the specified relay IP and port.
        
        Args:
            relay_ip (str): IP address of the relay.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data = struct.pack('fff', self.x, self.y, float(self.orientation))
        sock.sendto(data, (relay_ip, self.port))
        sock.close()
        print(self)

model = YOLO('/home/daniela/Desktop/VSSS/larc-vsss-2025/VSSS_modelM/runs/epoch80.pt') #load model

realFieldCoors = [[0, 0], #tl
                  [150, 0], #tr
                  [150, 130], #br
                  [0, 130]] # bl
# Array with last orientations
orientation_history = {}
centroid_history = deque(maxlen=5)  

global robots 
robots = {}

# Predefined robot ports for each identified pattern
predefined_ports = {
    1201: [7,8,9]  # Robot 1
    # Add more robots and their ports as needed
}

for port, patters in predefined_ports.items():
    robots[port] = RobotData(port=port, patterns = patters)  # Initialize each robot with its port

#Modify depending on actual environment
hsvRanges = { 
    'blue' : {'lower':[100, 172 , 154], 'upper': [136, 255, 255]},
    'yellow' : {'lower': [22, 0, 163], 'upper':[33, 255, 255] }
}

def auto_adjust_hsv(hsv_img, mask):
    """
    Dynamically adjusts HSV ranges based on the histogram of the H channel.
    
    Args:
        hsv_img (ndarray): HSV image.
        mask (ndarray): Binary mask for the region of interest.
    
    Returns:
        tuple: Lower and upper HSV bounds.
    """
    h_channel = hsv_img[:, :, 0] 
    masked_h = h_channel[mask > 0] 
    if len(masked_h) > 0: 
        lower_h = np.percentile(masked_h, 5) 
        upper_h = np.percentile(masked_h, 95)  

        lower_hsv = np.array([lower_h, 50, 50])
        upper_hsv = np.array([upper_h, 255, 255])

        return lower_hsv, upper_hsv
    else:
        return None, None
   
def get_color_centroid(img):
    """
    Detects the centroid of the largest colored region in the image.
    
    Args:
        img (ndarray): Input image.
    
    Returns:
        tuple: Centroid coordinates (x, y) and the detected color.
    """
    imgBlur = cv2.GaussianBlur(img, (7,7), 0)
    hsv_img = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)                                          

    max_area = 0
    centroid = None
    biggest_color = None
    for color, ranges in hsvRanges.items():
        lower = np.array(ranges['lower'])
        upper = np.array(ranges['upper'])

        mask = cv2.inRange(hsv_img, lower, upper)

        kernel = np.ones((5, 5), np.uint8)

        dilatedMask = cv2.dilate(mask, kernel, iterations=1)
        erotedMask = cv2.erode(dilatedMask, kernel, iterations=1)

        contours, _ = cv2.findContours(erotedMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_cnt)
            if area > max_area:
                cv2.imshow("Mask", mask)
                max_area = area
                M = cv2.moments(largest_cnt)
                if M['m00'] > 0:
                    cX = int(M["m10"] / M['m00']) 
                    cY = int(M['m01'] / M['m00']) 
                    centroid = (cX, cY)
                    biggest_color = color
    
    return centroid, biggest_color

def get_orientation(img, color_centroid):
    """
    Calculates the orientation of the robot based on its centroid and a color marker.
    
    Args:
        img (ndarray): Region of interest containing the robot.
        color_centroid (tuple): Centroid of the color marker.
    
    Returns:
        float: Orientation in radians.
    """
    bb_shape = img.shape
    if color_centroid != None:
        cX = float(bb_shape[0] / 2)
        cY = float(bb_shape[1] / 2)
        robot_centroid = (cX, cY)

        dx = robot_centroid[0] - color_centroid[0]
        dy = robot_centroid[1] - color_centroid[1]

        #return orientation in degrees; change depending on control needs
        orientation = (math.atan2(dy, dx)) #use math.degrees() % 360 for degrees use
        cv2.line(img, (int(color_centroid[0]), int(color_centroid[1])), (int(robot_centroid[0]), int(robot_centroid[1])), (0,255, 0), 2)
        return orientation
    else:
        return None

def bb_center_orien(results, img, H):
    """
    Processes bounding boxes detected by YOLO to calculate robot positions and orientations.
    
    Args:
        results (list): YOLO detection results.
        img (ndarray): Input image.
        H (ndarray): Homography matrix for perspective transformation.
    
    Returns:
        dict: Updated robot data.
    """
    global orientation_history, robots
    robot_coors = (0.0, 0.0)

    for res in results:
        boxes = res.boxes  

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            #confidence = box.conf[0].item()
            patternID = int(box.cls[0]) + 1  
            robot_id = None
            for port, robot in robots.items():
                if patternID in robot.patterns:
                    robot_id = port
                    break
            if( robot_id is None):
                print("No se detectó el patrón") 
                continue
            x_center = float((x1 + x2) / 2)
            y_center = float((y1 + y2) / 2)

            roi = img[int(y1):int(y2), int(x1):int(x2)]
            color_centroid, _ = get_color_centroid(roi)

            if color_centroid is not None:
                smoothed_centroid = robots[robot_id].moving_average_centroid(color_centroid)
                orien = get_orientation(roi, smoothed_centroid)             
                
                if orien is not None:
                    orien = round(float(orien), 4)
                    robots[robot_id].orientation_history.append(orien)
                    smoothed_orien = robots[robot_id].moving_average_orientation()
                    
                    robot_coors = np.array([[x_center, y_center]], dtype="float32")
                    robot_coors = np.array([robot_coors])
                    real_robot_coors = cv2.perspectiveTransform(robot_coors, H)[0][0]
                    last_robot_data = {"x": real_robot_coors[0], "y": real_robot_coors[1], "orientation": orien} # Historial de datos del robot  
                    
                    robots[robot_id].update(real_robot_coors[0], real_robot_coors[1], smoothed_orien)
                    print(f"Robot {robot_id}: {real_robot_coors[0]}, {real_robot_coors[1]}, {orien}")

                    cv2.circle(img, (int(x_center), int(y_center)), 2, (0, 0, 255), -1)
                    cv2.putText(img, f"ID: {robot_id}", (int(x_center), int(y_center) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return robots

'''def main():               
    cap = cv2.VideoCapture(2)
    cap.set(3, 640) #width
    cap.set(4, 480) #height

    H = getHomography(cap, realFieldCoors)
    matrix = warpChange()
    newH = autoGetHomography(realFieldCoors)

    #main loop
    while True:
        tpast = time.time()
        success, img = cap.read()

        if success:
            field = cv2.warpPerspective(img, matrix, (640, 480))


            cv2.imshow("Field", field)
            results = model(field)
            if results:
                res_img = results[0].plot()
                bb_center_orien(results, field, H)
                cv2.imshow("Model prediction", res_img)
                cv2.imshow("Video", field)
                cv2.imshow("Normal video", img)
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

if __name__ == '__main__':
    main()'''

