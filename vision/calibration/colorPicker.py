import cv2
import numpy as np

#Function for createTrackbars to work
def empty(value):
    pass

def autoHSVPicker():
    pass
#preprocess 
def colorPicker(window):
    """
    Creates trackbars for HSV range selection and processes video input to filter colors.
    
    Args:
        window (str): Name of the window where trackbars will be displayed.
    """
    #Trackbars creation
    cv2.createTrackbar("Hue Min", window, 1, 179, empty)
    cv2.createTrackbar("Hue Max", window, 179, 179, empty)
    cv2.createTrackbar("Sat Min", window, 0, 255, empty)
    cv2.createTrackbar("Sat Max", window, 255, 255, empty)
    cv2.createTrackbar("Val Min", window, 0, 255, empty)
    cv2.createTrackbar("Val Max", window, 255, 255, empty)

    #Video capture and settings
    video = cv2.VideoCapture(2)
    video.set(3, 640) #width
    video.set(4, 480) #Height
    video.set(10, 150) #brightness
    
    while True:
        ret, img = video.read()
        imgBlur = cv2.GaussianBlur(img, (7,7), 0)
        imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)


        h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
        h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
        s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
        s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
        v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
        v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

        lower = np.array([h_min, s_min, v_min]) 
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(imgHSV, lower, upper)

        kernel = np.ones((5, 5), np.uint8)

        dilatedMask = cv2.dilate(mask, kernel, iterations=1)
        erotedMask = cv2.erode(dilatedMask, kernel, iterations=1)
        cv2.imshow("Mascara eroted", erotedMask)
        result = cv2.bitwise_and(img, img, mask=erotedMask)

        print("h_min = ", h_min, " h_max = ", h_max, " Sat_min = ", s_min, " Sat_max = ", s_max, " Val_min = ", v_min, " Val_max = ", v_max)
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

#Window for trackbars 
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 240)
colorPicker("TrackBars")

