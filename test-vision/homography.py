import cv2
import numpy as np

CAMERA_HEIGHT = 200 #cm
clicked_points = []

def mouse_callback(event, x, y, _, __):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked: {x}, {y}")
        clicked_points.append((x, y))
    
def autoGetHomography(cap, realCoors, cameraCoors):

    pxCoors = np.array(cameraCoors, dtype = "float32")
    realCoors = np.array(realCoors, dtype = "float32")

    H, _ = cv2.findHomography(pxCoors, realCoors, cv2.RANSAC, 5.0)
    
    return H 


def getHomography(cap, realCoor):
    global clicked_points
    clicked_points = []

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", mouse_callback)

    while len(clicked_points) < 4: #<4 because 0 will also be a point (0,1,2,3) = len 4
        success, img = cap.read()
        if not success:
            continue
        cv2.putText(img, f"Click on Point {len(clicked_points) + 1} out of 4", (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
        
        for pt in clicked_points:
            cv2.circle(img, pt, 3, (0, 0, 255), -1)
        cv2.imshow("Calibration", img)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            raise Exception("Calibration aborted")
    
    cv2.destroyWindow("Calibration")

    pxCoors = np.array(clicked_points, dtype = "float32")
    realCoors = np.array(realCoor, dtype = "float32")

    H, _ = cv2.findHomography(pxCoors, realCoors, cv2.RANSAC, 5.0)
    
    return H 

