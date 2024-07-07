import cv2 as cv
from cv2 import aruco
import numpy as np

# Function to resize frame while maintaining aspect ratio
def resize_frame(frame, width):
    height = int(frame.shape[0] * (width / frame.shape[1]))
    return cv.resize(frame, (width, height))

# Camera calibration parameters (you need to calibrate your camera to get these values)
camera_matrix = np.array([[800, 0, 320],
                          [0, 800, 240],
                          [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion

# Create a dictionary of predefined ArUco markers
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Camera video stream
cap = cv.VideoCapture(0)

# Marker length in meters (change according to the size of your markers)
marker_length = 0.1


while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Resize frame
    frame = resize_frame(frame, 720)
    
    # Convert frame to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detect markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # Draw detected markers and display their IDs
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Estimate pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        
        for i in range(len(ids)):
            # Draw axis for each marker
            cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
            
            print('len marker ids:', len(ids))
            print('rvecs:', rvecs)
            print('tvecs:', tvecs)
            print('ids:', ids)
            print('=====')

    cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
