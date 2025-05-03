from ultralytics import YOLO
import torch
import cv2
import time
import math
import numpy as np
import json #if communication tells you he needs the info in json format
from homography import getHomography
import struct
import socket
from collections import deque


#PORT_IP = 1234 #change port for robot data communication
RELAY_IP = "192.168.0.171"  # Replace with your esp

#changes
model = YOLO('/home/alberto/Coding/LARCVSSS/larc-vsss-2025/VSSSModel/runs/detect/custom_VSSS_model/weights/best.pt')


realFieldCoors = [[0, 0], #tl
                  [150, 0], #tr
                  [150, 130], #br
                  [0, 130]] # bl

hsvRanges = {
    'blue' : {'lower':[108, 120, 0], 'upper': [130, 255, 206]}, #h_min =  95  h_max =  111  Sat_min =  122  Sat_max =  255  Val_min =  80  Val_max =  255
    'yellow' : {'lower': [18, 82, 0], 'upper':[28, 255, 255] } #Ah_min =  4  h_max =  55  Sat_min =  19  Sat_max =  196  Val_min =  51  Val_max =  255
}

    
#Communication python to esp32
def send_coordinates_robot(x, y, orientation, relay_ip, relay_port):
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
    if orientation != None and x != None:
        data = struct.pack('fff', x, y, float(orientation)) # Use 'e' for 16-bit floats 
        # Send the data
        sock.sendto(data, (relay_ip, relay_port))
        # Close the socket
        sock.close()

def auto_adjust_hsv(hsv_img, mask):
    """Ajusta dinámicamente los rangos HSV basándose en el histograma del canal H."""
    h_channel = hsv_img[:, :, 0]  # Extraer canal H
    masked_h = h_channel[mask > 0]  # Píxeles dentro de la máscara

    if len(masked_h) > 0:  # Verifica si hay píxeles en la máscara
        lower_h = np.percentile(masked_h, 5)  # Percentil inferior (5%)
        upper_h = np.percentile(masked_h, 95)  # Percentil superior (95%)

        # Ajustar los rangos dinámicamente (usando valores predeterminados para S y V)
        lower_hsv = np.array([lower_h, 50, 50])
        upper_hsv = np.array([upper_h, 255, 255])

        return lower_hsv, upper_hsv
    else:
        return None, None
    
def get_color_centroid(img):
    blurred = cv2.GaussianBlur(img, (7,7), 0)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_img = cv2.medianBlur(hsv_img, 5)  # Aplicar un filtro mediano para reducir ruido
    hsv_img = cv2.bilateralFilter(hsv_img, 9, 75, 75)  # Filtrado bilateral para preservar bordes

    max_area = 0
    centroid = None
    biggest_color = None
    cv2.imshow("Blurred passing", img)
    for color, ranges in hsvRanges.items():
        lower = np.array(ranges['lower'])
        upper = np.array(ranges['upper'])

        mask = cv2.inRange(hsv_img, lower, upper)

        # Ajuste dinámico de los valores HSV
        lower_hsv, upper_hsv = auto_adjust_hsv(hsv_img, mask)
        if lower_hsv is not None and upper_hsv is not None:
            mask = cv2.inRange(hsv_img, lower_hsv, upper_hsv)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
         # Aplicar morfología para limpiar la máscara
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Cerrar pequeños huecos
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Eliminar ruido pequeño

        if contours:
            #returns the biggest contour based on it's area
            largest_cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_cnt)
            if area > max_area:
                cv2.imshow("Mask", mask)
                max_area = area
                M = cv2.moments(largest_cnt)
                if M['m00'] > 0:
                    cX = int(M["m10"] / M['m00']) #X entre area
                    cY = int(M['m01'] / M['m00']) #Y entre area
                    centroid = (cX, cY)
                    biggest_color = color
    
    #if not contours found, return None
    return centroid, biggest_color

def get_orientation(img, color_centroid):
    bb_shape = img.shape
    #the img here is only used for debug (to see the orientation line)
    if color_centroid != None:
        cX = float(bb_shape[0] / 2)
        cY = float(bb_shape[1] / 2)
        robot_centroid = (cX, cY)

        dx = robot_centroid[0] - color_centroid[0]
        dy = robot_centroid[1] - color_centroid[1]
        #print(f"Robot: {robot_centroid}\n Color: {color_centroid}")
        #return orientation in degrees; change depending on control needs
        orientation = math.degrees(math.atan2(dy, dx)) % 360 #use math.degrees() % 360 for degrees use
        cv2.line(img, (int(color_centroid[0]), int(color_centroid[1])), (int(robot_centroid[0]), int(robot_centroid[1])), (0,255, 0), 2)
        return orientation
    else:
        return None

    
# Define una lista para almacenar las últimas orientaciones
orientation_history = {}
centroid_history = deque(maxlen=5)  # Mantén un historial de los centroides

def moving_average_centroid(centroid, window_size=5):
    centroid_history.append(centroid)
    avg_x = sum([c[0] for c in centroid_history]) / len(centroid_history)
    avg_y = sum([c[1] for c in centroid_history]) / len(centroid_history)
    return (avg_x, avg_y)

# Función para calcular la media móvil
def moving_average(orientation_list, window_size=5):
    return sum(orientation_list) / len(orientation_list)  # Promedio de los valores disponibles
    

# Modifica la función bb_center_orien para incluir la media móvil
def bb_center_orien(results, img, H):
    global orientation_history
    robot_orien = 0
    robot_coors = (0.0, 0.0)

    for res in results:
        boxes = res.boxes  # Variable con todas las bb detectadas en el frame
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            track_id = box.id  # ID de seguimiento
            class_id = int(box.cls[0]) + 1  # Índice de la clase detectada

            # Centro del robot
            x_center = float((x1 + x2) / 2)
            y_center = float((y1 + y2) / 2)

            # ROI para calcular la orientación
            roi = img[int(y1):int(y2), int(x1):int(x2)]
            color_centroid, _ = get_color_centroid(roi)

            if isinstance(track_id, torch.Tensor):
                track_id = int(track_id.item())
            if color_centroid is not None:
                smoothed_centroid = moving_average_centroid(color_centroid)
                
                orien = get_orientation(roi, smoothed_centroid)
                
                if orien is not None and isinstance(track_id, int):
                    orien = round(float(orien), 4)
                    # Agregar orientación al historial del robot
                    print(f"Orien {track_id}: {orien}")
                    # Obtener coordenadas reales
                    robot_coors = np.array([[x_center, y_center]], dtype="float32")
                    robot_coors = np.array([robot_coors])
                    real_robot_coors = cv2.perspectiveTransform(robot_coors, H)[0][0]

                    # Enviar coordenadas y orientación suavizada al robot
                    #
                    #send_coordinates_robot(real_robot_coors[0], real_robot_coors[1], orien, RELAY_IP, 1201)
                    print(f"Robot {track_id}: {real_robot_coors[0]}, {real_robot_coors[1]}")

                    # Dibujar el centro del robot
                    x_center = int(x_center)
                    y_center = int(y_center)
                    cv2.circle(img, (x_center, y_center), 2, (0, 0, 255), -1)
                    

                
def main():               
    cap = cv2.VideoCapture(2)
    cap.set(3, 640) #width
    cap.set(4, 480) #height

    H = getHomography(cap, realFieldCoors)

    #main loop
    while True:
        tpast = time.time()
        success, img = cap.read()

        if success:
            results = model.track(source=img, persist=True, show=False )
            #results = model.predict(img)
            if results:
                res_img = results[0].plot()
                bb_center_orien(results, img, H)
                cv2.imshow("Model prediction", res_img)
                cv2.imshow("Video", img)
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
    main()