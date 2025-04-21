from ultralytics import YOLO
import cv2
import time
import math
import numpy as np
import json #if communication tells you he needs the info in json format

#changes
model = YOLO('/home/daniela/Desktop/VSSS/larc-vsss-2025/VSSSModel/runs/detect/custom_VSSS_model/weights/best.pt')

cap = cv2.VideoCapture(2)
cap.set(3, 640) #width
cap.set(4, 480) #height

hsvRanges = {
    'blue' : {'lower':[95, 122, 80], 'upper': [111, 255, 255]}, #h_min =  95  h_max =  111  Sat_min =  122  Sat_max =  255  Val_min =  80  Val_max =  255
    'yellow' : {'lower': [4, 19, 51], 'upper':[55, 196, 255] } #h_min =  4  h_max =  55  Sat_min =  19  Sat_max =  196  Val_min =  51  Val_max =  255
}

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
                cv2.imshow("Mask", mask)
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
def bb_center_orien(results, img):
    robots_data = {"robots" : []}
    robot_orien = 0
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
                print(f"Robot {class_id}, orientation: {orien}")

                '''#communicate position and orientation for each robot
                robot_data = {
                    'id' : class_id,
                    'position' : [x_center, y_center],
                    'orientation' : orien if orien != None else 0.0
                }
                robots_data['robots'].append(robot_data)'''

                #make them int for cv2.circle
                x_center = int(x_center)
                y_center = int(y_center) 
                #debug
                cv2.circle(img, (x_center, y_center), 2, (0,0,255), -1)
                #MEJOR USA LA HOMOGRAFÍA
                #cX, cY = pxConvertor(x_center, y_center)
                #cv2.putText(res_img, f"({real_robot_coors[0]:.1f}, {real_robot_coors:.1f})", 
                        #((x_center), (y_center)), 
                        #cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            
#PLAN B *NOT TOTALLY FUNCTIONAL*
def detect_orientation(img, x11, y11, x22, y22):
    x1, y1, x2, y2 = map(int, [x11, y11, x22, y22])
    roi = img[y1:y2, x1:x2]

    # Verificar que el ROI no esté vacío
    if roi.size == 0:
        print("ROI vacío o inválido")
        return None

    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.GaussianBlur(gray_roi, (3, 3), 0)
    thresh = cv2.adaptiveThreshold(gray_roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                               cv2.THRESH_BINARY_INV,13, 2) #13,2
    cv2.imshow("thresh", thresh)
    # Operaciones morfológicas
    # 2. Operaciones morfológicas más robustas
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=2)
    #cleaned = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2) 
    cv2.imshow("Cleaned", cleaned)
    # Detección de contornos
    contours, hierarchy = cv2.findContours(cleaned, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter out small contours (likely noise)
    min_contour_area = 5  # Adjust based on your marker size
    valid_contours = []
    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area > min_contour_area:
            # Check if contour has parent (to find inner markers)
            if hierarchy[0][i][3] != -1:  
                valid_contours.append(cnt)

    # Debug visualization
    debug_img = roi.copy()
    cv2.drawContours(debug_img, valid_contours, -1, (0,255,0), 1)
    cv2.imshow("Valid Contours", debug_img)
    
    # Dibujar contornos detectados en la ROI gris
    gray_with_contours = gray_roi.copy()
    cv2.drawContours(gray_with_contours, contours, -1, (255, 255, 255), 2)
    cv2.imshow("Contornos detectados en gray", gray_with_contours)
    print(f"shape: {gray_with_contours.shape}")
    print(f"Contornos VALIDOS: {len(valid_contours)}")
    print(f"Contornos detectados: {len(contours)}")
    # Dibujar contornos detectados en la ROI
    roi_with_contours = roi.copy()
    cv2.drawContours(roi_with_contours, contours, -1, (0, 255, 0), 2)
    cv2.imshow("Contornos detectados en ROI", roi_with_contours)

    # Calcular el centro del robot (bounding box)
    robot_cx = int((x1 + x2) / 2)
    robot_cy = int((y1 + y2) / 2)
    # Dibujar el centro del robot
    cv2.circle(img, (robot_cx, robot_cy), 2, (0, 0, 255), -1)
    
    contornos = list(contours)
    # Calcular orientación si hay suficientes etiquetas
    if len(contornos) >= 2:
        contornos.sort(key=lambda c: cv2.boundingRect(c)[0])  # Ordenar por coordenada x

        # Etiquetas frontales
        front_centers = []
        for val_cnt in contornos[1:3]:
            M = cv2.moments(val_cnt)
            if M['m00'] == 0:
                continue
            front_centers.append((
                int(M['m10'] / M['m00']) + x1,
                int(M['m01'] / M['m00']) + y1
            ))
        if len(front_centers) < 2:
            print("No se detectaron suficientes etiquetas frontales")
            return None

        # Calcular punto medio frontal
        front_cx = int((front_centers[0][0] + front_centers[1][0]) / 2)
        front_cy = int((front_centers[0][1] + front_centers[1][1]) / 2)
        # Dibujar el punto medio frontal
        cv2.circle(roi_with_contours, (front_cx - x1, front_cy - y1), 3, (255, 0, 0), -1)
        cv2.imshow("ROI with front center", roi_with_contours)
        cv2.line(img, (robot_cx, robot_cy), (front_cx, front_cy), (0,255, 0), 2)
        # Calcular orientación
        dx = front_cx - robot_cx
        dy = front_cy - robot_cy

        orientation = math.degrees(math.atan2(dy, dx)) % 360 # devuelve todos positivos de acuerdo a 0 a 360 si pones: math.degrees() % 360, 

        print(f"Orientación calculada: {orientation}")
        return orientation

    print("No se detectaron suficientes etiquetas para calcular la orientación")
    return None

#main loop
while True:
    tpast = time.time()
    success, img = cap.read()

    if success:
        results = model.predict(img)
        if results:
            res_img = results[0].plot()
            #center and orientation for each bb detected
            bb_center_orien(results, res_img)
            #comunicar datos de robots a control
            #send_coordinates(x_center, y_center, RELAY_IP, PORT_IP)'''
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