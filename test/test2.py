import cv2 as cv
from cv2 import aruco
import numpy as np
import os
from objloader import OBJ  # Asumiendo que tienes esta clase definida

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


# Marker length in meters (change according to the size of your markers)
marker_length = 0.05

# Load 3D model from OBJ file
scale_factor = 0.005
dir_name = os.getcwd()
obj = OBJ(os.path.join(dir_name, 'models/wolf.obj'), swapyz=True, dz=0.15, marker_length=marker_length,scale_factor=0.0002)
# obj = OBJ(os.path.join(dir_name, 'models/rat.obj'), swapyz=True, dz=0.15, marker_length=marker_length,scale_factor=scale_factor)
# obj = OBJ(os.path.join(dir_name, 'models/fox.obj'), swapyz=True, dz=0.15, marker_length=marker_length,scale_factor=0.0015)
# obj = OBJ(os.path.join(dir_name, 'models/chair.obj'), swapyz=True, dz=0.15, marker_length=marker_length,scale_factor=0.004)
# obj = OBJ(os.path.join(dir_name, 'models/sheet.obj'), swapyz=True, dz=0.15, marker_length=marker_length,scale_factor=0.015)
# obj = OBJ(os.path.join(dir_name, 'models/chess_pawn.obj'), swapyz=True, dz=0.0, marker_length=marker_length,scale_factor=0.01)
# obj = OBJ(os.path.join(dir_name, 'models/pegasus.obj'), swapyz=True, dz=0.0, marker_length=marker_length,scale_factor=0.05)
obj.print_vertices()

# Camera video stream
cap = cv.VideoCapture(0)

# Main loop for video capture and AR visualization
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
    
    # Load the vertices and faces of the 3D model
    vertices = np.array(obj.vertices)
    
    # Draw detected markers and display their IDs
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Estimate pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        
        for i in range(len(ids)):
            # Draw axis for each marker
            cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
            
            if ids[i] == 35:
                # Project 3D vertices to image plane
                imgpts, _ = cv.projectPoints(vertices, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
                imgpts = np.int32(imgpts).reshape(-1, 2)
                
                # Draw the vertices as points
                for pt in imgpts:
                    frame = cv.circle(frame, tuple(pt), 3, (0, 0, 255), -1)
                
                # Draw the faces of the model
                for face in obj.faces:
                    points = [imgpts[vertex-1] for vertex in face[0]]
                    points = np.array(points, dtype=np.int32)
                    cv.polylines(frame, [points], True, (255, 255, 0), 2)  # Draw each face as a closed polygon

    cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
