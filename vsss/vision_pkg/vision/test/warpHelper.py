import cv2
import numpy as np

cap = cv2.VideoCapture(2)
cap.set(3, 640)
cap.set(4, 480)

#TODO: Convert into a function to use it with the images from the camera topic
# and use a new function in the general vision class to get the warped image of the field and maybe upload it into a topic

def warp_change(src_pts, dst_pts, w, h):
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(frame, M, (w, h))
    return warped

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("No frame captured.")
        break

    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img_blur = cv2.GaussianBlur(img_gray, (7, 7), 0)
    _, thresh = cv2.threshold(img_blur, 175, 255, cv2.THRESH_BINARY)
    cv2.imshow("Thresholded Image", thresh)
    cnts,_ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    height, width = frame.shape[:2]
    print("Frame dimensions:", width, "x", height)
    img_copy = frame.copy()

    maxArea = 0
    maxContour = None
    X, Y, W, H = 0, 0, 0, 0
    for cnt in cnts:
        x, y, w, h = cv2.boundingRect(cnt)
        rectArea = w * h
        #avoid hole image contour
        if x <= 0 or y <= 0 or x + w >= width or y + h >= height:
            continue

        if rectArea > maxArea:
            maxArea = rectArea
            maxContour = cnt
            X, Y, W, H = cv2.boundingRect(maxContour)

    dst_width, dst_height = 640, 480
    if maxContour is not None:
        #use this or warp perspective
        cv2.rectangle(img_copy, (X, Y), (X + W, Y + H), (255, 0, 0), 2)
        cv2.drawContours(img_copy, [maxContour], -1, (0, 255, 0), 2, cv2.LINE_AA)
        src_pts = np.float32([ 
            [X, Y], 
            [X + W, Y], 
            [X + W, Y + H], 
            [X, Y + H] 
        ])
        dst_pts = np.float32([ 
            [0, 0], 
            [dst_width, 0], 
            [dst_width, dst_height], 
            [0, dst_height]
        ])
        warped = warp_change(src_pts, dst_pts, 640, 480)
        cv2.imshow("Warped", warped)

    cv2.imshow("Contours", img_copy)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()