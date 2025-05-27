import cv2
import numpy as np

CAMERA_HEIGHT = 200 
clicked_points = []
width = 640
height = 480
objectivePoints = np.float32([[0, height], [0,0], [width, 0], [width, height]])

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


def getHomography(cap, realCoor):
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
    
    while len(clicked_points) < 4: 
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

def autoGetHomography(realCoor):
    """
    Automatically computes the homography matrix using predefined pixel coordinates.
    
    Args:
        realCoor (list): List of real-world coordinates corresponding to the predefined pixel points.
    
    Returns:
        ndarray: Homography matrix mapping pixel coordinates to real-world coordinates.
    """
    pxCoors = np.array([(0,0), (width, 0), (width, height), (0, height)])
    realCoors = np.array(realCoor, dtype = "float32")

    H, _ = cv2.findHomography(pxCoors, realCoors, cv2.RANSAC, 5.0)
    
    return H


def warpChange():
    """
    Computes the perspective transformation matrix for warping the image.
    
    Returns:
        ndarray: Perspective transformation matrix.
    """
    clickedOriginal = np.array(clicked_points, dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(clickedOriginal, objectivePoints)
    return matrix