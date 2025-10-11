#!/usr/bin/env python3

import cv2
from .vision_constants import (
    YOLO_LOCATION,
    CONF_THRESH,
)

import rclpy 
from rclpy.node import Node
from ultralytics import YOLO
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d.euler as euler
import math
import numpy as np
import torch
import os
from typing import List, Dict
# from vision_pkg.vision.utils.select_robot import select_robot
 
"""
    Node to take camera input and detect robots position and orientation 
    as well as ball position in real field coordinates
"""

import ament_index_python.packages as pkg

# Intentar obtener el directorio de datos del paquete instalado
package_share_dir = pkg.get_package_share_directory('vision')
luts_path = os.path.join(package_share_dir, "utils", "LUTs")
utis_path = os.path.join(package_share_dir, "utils")

# Construir ruta completa del modelo YOLO
yolo_model_path = os.path.join(package_share_dir, YOLO_LOCATION)

if not os.path.exists(yolo_model_path):
    base_path = os.path.dirname(os.path.abspath(__file__))
    yolo_model_path = os.path.join(base_path, "..", YOLO_LOCATION)


device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

colors = {
    "green": np.load(os.path.join(luts_path,"lut_green.npy" )),
    "blue": np.load(os.path.join(luts_path, "lut_blue.npy")),
    "pink": np.load(os.path.join(luts_path, "lut_pink.npy")),
    "red": np.load(os.path.join(luts_path, "lut_red.npy")),
    "yellow": np.load(os.path.join(luts_path, "lut_yellow.npy")),
    "darkblue": np.load(os.path.join(luts_path, "lut_darkblue.npy"))
}

draw_colors = {
    "green": (0, 255, 0),
    "yellow": (0, 255, 255),
    "blue": (255, 255, 0),
    "darkblue": (255, 0, 0),
    "pink": (255, 0, 255),
    "red": (0, 0, 255)
}

patterns = {
    ("darkblue", "green", "red"): 133,
    ("darkblue", "blue", "red"): 26,
    ("darkblue", "red", "green"): 3,
    ("darkblue", "blue", "green"): 1,
    ("darkblue", "pink", "green"): 5,
    ("darkblue", "red", "blue"): 2,
    ("darkblue", "green", "blue"): 7,
    ("darkblue", "pink", "blue"): 8,
    ("darkblue", "green", "pink"): 9,
    ("darkblue", "blue", "pink"): 10,
    ("yellow", "green", "red"): 11,
    ("yellow", "blue", "red"): 12,
    ("yellow", "red", "green"): 13,
    ("yellow", "blue", "green"): 14,
    ("yellow", "pink", "green"): 2,
    ("yellow", "red", "blue"): 16,
    ("yellow", "green", "blue"): 17,
    ("yellow", "pink", "blue"): 1,
    ("yellow", "green", "pink"): 2,
    ("yellow", "blue", "pink"): 1,
}

# threshold = 5.0 #thought in cm, but see if this should be in pixels.

class robot:
    def __init__(self, id : str, team: str, location, angle):
        self.id = id
        self.location = location
        self.angle = angle
        self.team = team
        self.relative_distance = None
        self.angle_window = []
        # self.pos_window = []
    
    def __eq__(self, other):
        return self.id == other.id and self.location == other.location and self.angle == other.angle and self.team == other.team
    
    def __hash__(self):
        return hash((self.id, tuple(self.location), self.angle, self.team))

    def select_robot(self, detected_robots, tf_helper):
        '''
        Function to get the robots id's based on the last frame detections
        using aspects such as: relative distance (now -> past), team, id, secondary color.
        
        Should be called for both color teams (yellow and blue)
        
        Returns a list of the detected robots in a frame
        '''
        #compare the actual robot with the list of past robots;
        possibilities = []
        same_id = []
        same_color = []
        same_team = []
        not_same = []

        for detected_robot in detected_robots:
            distance = get_eucladian(detected_robot.location, self.location) #check number sign, if negative, use abs
            detected_robot.relative_distance = distance
            if detected_robot.id == self.id:
                same_id.append(detected_robot)
            elif detected_robot.team == self.team:
                same_team.append(detected_robot)
            else:
                not_same.append(detected_robot)

        same_id_ordered = sorted(same_id, key=lambda r:r.relative_distance) #check if this focus is correct, appending the four hole arrays?
        same_color_ordered = sorted(same_color, key=lambda r:r.relative_distance)
        same_team_ordered = sorted(same_team, key=lambda r:r.relative_distance)
        not_same_ordered = sorted(not_same, key=lambda r:r.relative_distance)
        
        possibilities.append(same_id_ordered)    
        ids_1 = [robot.id for robot in same_id_ordered]
        # self.get_logger().info(f"ID ID: {ids_1}")

        possibilities.append(same_color_ordered)
        ids_2 = [robot.id for robot in same_color_ordered]
        # self.get_logger().info(f"ID color: {ids_2}")

        possibilities.append(same_team_ordered)
        ids_3 = [robot.id for robot in same_team_ordered]
        # self.get_logger().info(f"ID team: {ids_3}")

        possibilities.append(not_same_ordered)
        ids_4 = [robot.id for robot in not_same_ordered]
        # self.get_logger().info(f"ID nor same ordered: {ids_4}")

        selected_robot = None
        for possibility in possibilities:
            if len(possibility) > 0:
                # self.get_logger().warn("selected possibility")
                #possibility[0] should be the most optimal
                #TODO: See if using mre than the first robot could help.
                selected_robot = possibility[0]
                break
                    

        if selected_robot is not None:
            self.location = selected_robot.location
            if selected_robot.angle is not None:
                self.angle = selected_robot.angle
            # self.get_logger().warn(f"Selected a robot: {selected_robot}")

            yaw = math.radians(self.angle)
            pitch, roll = 0.0, math.pi
            tf_helper("robot" + str(self.id) + "_base_link", self.location[0], self.location[1], roll, pitch, yaw)

        return selected_robot



yellow_team = [1, 2] #4, 6 #19, 18
darkblue_team = []
detected_robots = [] #should always have a maximum of six robots
past_robots = [] #used

kernel_size = 10
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

orange = np.load(os.path.join(luts_path, "lut_orange.npy"))

width = 1280
height = 720

objectivePoints = np.float32([[0, 0], [0,height], [width, height], [width, 0]])
real_field_coors = [[0,0],
                    [0, 130],
                    [150, 130],
                    [150, 0]]

clicked_points = []
coors_clicked = []

robot_capacity = len(yellow_team)

def mouse_callback(event, x, y, _, __):
    """
    Callback function to capture mouse clicks and store the clicked points.
    
    Args:
        event (int): Type of mouse event (e.g., left button click).
        x (int): X-coordinate of the mouse click.
        y (int): Y-coordinate of the mouse click.
        _ (int): Unused parameter.
        __ (int): Unused parameter.
    """
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked: {x}, {y}")
        clicked_points.append((x, y))

def getHomography(img, realCoor):
    """
    Computes the homography matrix based on user-clicked points and real-world coordinates.
    
    Args:
        cap (cv2.VideoCapture): Video capture object for live feed.
        realCoor (list): List of real-world coordinates corresponding to the clicked points.
    
    Returns:
        ndarray: Homography matrix mapping pixel coordinates to real-world coordinates.
    """
    global clicked_points
    clicked_points = []

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", mouse_callback)

    while(len(clicked_points) < 4):
        img_copy = img.copy()
        cv2.putText(img_copy, f"Click on Point {len(clicked_points) + 1} out of 4", (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)

        for pt in clicked_points:
            cv2.circle(img_copy, pt, 2, (0, 0, 255), -1)
        cv2.imshow("Calibration", img_copy)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise Exception("Calibration aborted")
    
    cv2.destroyWindow("Calibration")

    pxCoors = np.array(clicked_points, dtype=np.float32)
    realCoors = np.array(realCoor, dtype=np.float32)

    H, _ = cv2.findHomography(pxCoors, realCoors, cv2.RANSAC, 5.0)
    np.save("homography.npy", H)
    
    matrix = cv2.getPerspectiveTransform(pxCoors, objectivePoints)
    np.save("persMatrix.npy", matrix)

    return H, matrix

def get_eucladian(pt1, pt2):
    """
    Get the eucladian distance between two points
    """
    # pt1 is a tuple of two elements
    a = np.array([pt1[0], pt1[0]])
    b = np.array([pt2[0], pt2[1]])
    distance = np.linalg.norm(a - b)
    return distance


class CameraDetections(Node):
    def __init__(self):
        super().__init__('camera_detections')
        self.video_id = self.declare_parameter("Video_ID", 2)
        # self.get_logger().info("Camera id taken")
        self.cap = cv2.VideoCapture(self.video_id.value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.yolo_model = YOLO(yolo_model_path)  
        self.yolo_model.to(device)
        self.get_logger().info("DEVICE: " + str(device))
        try:
            self.homography = np.load(os.path.join(utis_path, "homography.npy"))
            self.perspectiveMatrix = np.load(os.path.join(utis_path, "persMatrix.npy"))
        except (FileNotFoundError, FileExistsError):
            self.get_logger().info("Homography || persmatrix not found, calibrate it")
            self.homography = None
            self.perspectiveMatrix = None
        self.image = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_center = None
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Starting model node\general vision node")

    def timer_callback(self):


        start = time.perf_counter()




        """
        Captures frames, warps them and processess model detections in the warped image.
        """
        if rclpy.ok():
            success, frame = self.cap.read()
            if not success:
                self.get_logger().info("No frame captured.")
                return

            if self.homography is None or self.perspectiveMatrix is None:
                self.get_logger().info("Starting calibration process...")
                try:
                    self.homography, self.perspectiveMatrix = getHomography(frame, real_field_coors)
                    self.get_logger().info("Calibration completed")
                except Exception as e:
                    self.get_logger().error(f"Failed calibration: {e}")
                    return
                
            # Warpea la imagen antes de procesar
            warped_img = cv2.warpPerspective(frame, self.perspectiveMatrix, (width, height))
            self.image = warped_img
            self.model_use()
            self.ball_detection(warped_img)
            elapsed = time.perf_counter() - start
            try:
                self.exec_time_pub.publish(Float32(data=elapsed))
            except Exception:
                # avoid crashing the timer if publishing fails
                pass


        
    
    def warp_image(self, data):
        '''
        Gets the warped image if homography file is available, if not, 
        will execute homography calibration process
        '''
        if self.homography is not None:
            # print(f"Homography -> {self.homography}")
            warped_img = cv2.warpPerspective(data, self.perspectiveMatrix, (640, 480)) #see if it's better to have 640, 480
                  
        else:
            self.homography, self.perspectiveMatrix = getHomography(data, real_field_coors)

    def tf_helper(self, id, x, y, roll, pitch, yaw):
        '''
        Sends the data transforms (position and angle)
        '''
        t = TransformStamped()
        self.get_logger().warn("Sending tf's")
        qx, qy, qz, qw = euler.euler2quat(yaw, pitch, roll, axes='sxyz') #roll, pitch, yaw = radians
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "lower_left_corner"
        t.child_frame_id = id
        t.transform.translation.x = float(x) 
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def image_to_field(self, x_img, y_img):
        '''
        Uses homography and perspective matrix to get the actual field values in cm
        '''
        # Si la imagen ya está warpeada, no aplicar la inversa de perspectiveMatrix
        pt_img = np.array([[[x_img, y_img]]], dtype=np.float32)
        inverse_perspective = np.linalg.inv(self.perspectiveMatrix)
        pt_original = cv2.perspectiveTransform(pt_img, inverse_perspective)
        pt_transformed = cv2.perspectiveTransform(pt_original, self.homography)
        x_field, y_field = pt_transformed[0][0]  # Coordenadas reales del campo
        # x_field, y_field = pt_field[0][0]
        return x_field, y_field

    def get_info_robot(self, img, position):
        '''
        
        '''
        scale = 6
        img = cv2.resize(img, None, fx=scale, fy=scale)

        h, w, _ = img.shape
        img_center = (w // 2, h // 2)

        img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        u = img_yuv[:, :, 1]
        v = img_yuv[:, :, 2]

        #get the areas of the pattern colors 
        centers = []
        for color_name, lut in colors.items():
            mask = lut[v, u]
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 2000:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        centers.append((cx, cy, color_name, area))

                        cv2.drawContours(img, [cnt], -1, draw_colors[color_name], 2)
                        cv2.circle(img, (cx, cy), 5, draw_colors[color_name], -1)

        darkblue_yellow = []
        for (x, y, c, area) in centers:
            if c in ["darkblue", "yellow"]:
                 darkblue_yellow.append((c, area))
                 
        #get the team from the largest area color contour
        try:
            team = sorted(darkblue_yellow, key= lambda x: x[1], reverse=True)[0][0]
        except:
            team = "yellow"

        #get the secondary two colors
        filtered_centers = [(x, y, c, area) for (x, y, c, area) in centers if c not in ["darkblue", "yellow"]]
        #get the largesr color areas to get the two actual secondary plates and avoid noise
        id_colors = sorted(filtered_centers, key=lambda x: x[3], reverse=True)[:2]
        
        angle = None
        if len(id_colors) == 2:
            (x1, y1, c1, a1), (x2, y2, c2, a2) = id_colors

            mid_x = (x1 + x2) // 2
            mid_y = (y1 + y2) // 2

            cv2.line(img, img_center, (mid_x, mid_y), (255, 255, 255), 2)
            cv2.circle(img, (mid_x, mid_y), 6, (255, 255, 255), -1)

            #get robot angle based on two secondary color plates
            dx = mid_x - img_center[0]
            dy = mid_y - img_center[1]
            angle = math.degrees(math.atan2(dy, dx))
            self.get_logger().info(f"Angle: {angle}")
            cv2.imshow("Orientacion", img)
            cv2.waitKey(1)
            
            if angle is not None:
                #x and y components of angles
                vx = math.cos(math.radians(angle))
                vy = math.sin(math.radians(angle))

                left_marker, right_marker = None, None
                for (x, y, c, _) in [(x1, y1, c1, a1), (x2, y2, c2, a2)]:
                    #get secondary center position relative to robot center (img center)
                    rel_x = x - mid_x
                    rel_y = -(y - mid_y) # -  for y to be in the same refernece system as the image
                    #use of cross product 2D to get 
                    cross = vx * rel_y - vy * rel_x
                    if cross > 0:
                        left_marker = c
                    else:
                        right_marker = c

                robot_id = patterns.get((team, left_marker, right_marker), None)
                self.get_logger().info("ID -> " + str(robot_id))
                # self.get_logger().info(" -> " + len(detected_robots))
                #initial list of detected robots
                if len(past_robots) < robot_capacity: #change number when testing
                    in_past = any(past_robot.id == robot_id for past_robot in past_robots)
                    if robot_id is not None and robot_id in yellow_team and not in_past:
                        robot_detected = robot(robot_id, team, position, angle)
                        past_robots.append(robot_detected)
                        self.get_logger().info("ID metido-> " + str(robot_id))
                        return
                    elif robot_id is not None and robot_id in yellow_team and in_past:
                        bot = robot(robot_id, team, position, angle)
                        return bot
                    else:
                        return #check return of Nonetype
                else:
                    self.get_logger().info("IDs llenos -> ")
                    if robot_id is not None:
                    #no matter if id detected or not, select robot will handle that
                        new_robot = robot(robot_id, team, position, angle)
                        return new_robot
                    #function will classify robot id based on past frame, and send it's transform
                    #refresh detected robots list
                    else: 
                        new_robot = robot(None, team, position, angle)
                        return new_robot

        elif len(past_robots) == robot_capacity:
            robot_detected = robot(None, team, position, None)
            return robot_detected   
        else:
            robot_detected = robot(None, None, position, None)
            return
        
    

    def model_use(self):
        '''
        Gets the robots bounding box and positions
        return NONE
        '''
        if self.image is None:
            self.get_logger().warn("No image received yet (model part)")
            return None
        frame = self.image.copy()
        results = self.yolo_model(frame, verbose=False, classes=0)
        if results is not None:
            robots_present = [] #robots per frame
            for result in results:
                count = len(result.boxes)
                self.get_logger().warn(f"MODEL IDENTIFIED ROBOTS = {len(result.boxes)}")
                for box in result.boxes:
                    x, y, w, h = [round(i) for i in box.xywh[0].tolist()]
                    confidence = box.conf.item()
                    
                    if confidence > CONF_THRESH:
                        #Robot position 
                        x1 = int(x - w / 2)
                        y1 = int(y - h / 2)
                        x2 = int(x + w / 2)
                        y2 = int(y + h / 2)
                        roi = frame[y1:y2, x1:x2]
                        
                        x_center = (x1 + x2) / 2 
                        y_center = (y1 + y2) / 2

                        x_field, y_field = self.image_to_field(x_center, y_center)
                        x_cm = x_field / 100
                        y_cm = y_field / 100

                        # Dibuja el bounding box
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        #Convert to field coordinates
                        # text = f"{x_cm}, {y_cm}"
                        # cv2.putText(frame, text, (int(x_center), int(y_center)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        #GET information of the robots (uses roi and robot position--------------------------------------------------------
                        robot_info = self.get_info_robot(roi, [x_cm, y_cm]) #fill table 
                        self.get_logger().info(f"{type(robot_info)}")
                        if (robot_info is not None):
                            robots_present.append(robot_info)

                nones = any(robot is None for robot in robots_present)
                if len(result.boxes) != 0 and not nones:
                    selected_robots = []
                    self.get_logger().info(f"PAST LENE -> {len(selected_robots)}")
                    for past_robot in past_robots:
                        self.get_logger().info(f"Actualizando past {past_robot.id}")
                        self.get_logger().info(f"ROBOTS_PRESENT -> {len(robots_present)}")
                        robots_present = list(set(robots_present) - set(selected_robots))
                        self.get_logger().info(f" RESULTS -> {len(robots_present)}")
                        selected_robot = past_robot.select_robot(robots_present, self.tf_helper)
                        selected_robots.append(selected_robot)
                    robots_present.clear()
                    selected_robots.clear()

                # cv2.putText(frame, F"COUNT: {count}", (300, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Model", frame)


    '''toma el id y lo mete a la lista, si es que no hay uno ya, y eso despues dera de la lista del pasado
    la función de arriba va tanto para azul como amarillo'''

    def ball_detection(self, img):
        frame = img.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # Bajar brillo
        v = np.clip(v - 40, 0, 255)

        hsv_darker = cv2.merge([h, s, v])
        frame = cv2.cvtColor(hsv_darker, cv2.COLOR_HSV2BGR)

        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        U = yuv[:, :, 1]
        V = yuv[:, :, 2]

        mask = orange[V, U]

        # Filtrado
        # quita puntitos de ruido
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        # rellena agujeros pequeños dentro de la pelota
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        # suavizado pequeño para bordes más lisos
        mask = cv2.medianBlur(mask, 5)
        # cv2.imshow("Mask", mask)
        cv2.waitKey(1)
        
        # encontrar la pelota
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        possible_ellipses = []

        for cnt in contours:
            if len(cnt) >= 4: 
                area = cv2.contourArea(cnt)
                if area > 10 and area < 10000:  # se ajusta dependiendo del tamaño esperado
                    ellipse = cv2.fitEllipse(cnt)
                    possible_ellipses.append(ellipse)
                    
        chosen_ellipse = None

        if possible_ellipses:
            if self.last_center is None:
                chosen_ellipse = max(possible_ellipses, key=lambda e: np.pi * (e[1][0] / 2) * (e[1][1] / 2))
            else:
                def distance(e):
                    center = e[0]
                    return np.linalg.norm(np.array(center) - np.array(self.last_center))
                
                chosen_ellipse = min(possible_ellipses, key=distance)
            
        if chosen_ellipse is not None:
            cv2.ellipse(frame, chosen_ellipse, (0, 255, 0), 2)
            self.last_center = chosen_ellipse[0]
            (x, y) = self.last_center
            cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
            self.get_logger().info("Ball center: " + str(self.last_center))

            real_x, real_y = self.image_to_field(x, y)
            # Mostrar las coordenadas reales junto al círculo verde
            text = f"({real_x:.1f}, {real_y:.1f})"
            cv2.putText(frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cm_x  = real_x / 100 
            cm_y = real_y / 100
            self.tf_helper("sphere_link", cm_x, cm_y, 0.0, 0.0, 0.0)
        else:
            self.last_center = None
            self.get_logger().info("Ball not detected")
        # cv2.imshow("YES", frame)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = CameraDetections()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()