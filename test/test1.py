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

# Displacement values
dx = -0.05  # Displacement along x-axis
dy = -0.05  # Displacement along y-axis
dz = 0.15 # Displacement along z-axis

CUBE_VERTICES = np.float32([
    [dx, dy, dz], [dx, dy + marker_length, dz], [dx + marker_length, dy + marker_length, dz], [dx + marker_length, dy, dz],
    [dx, dy, dz - marker_length], [dx, dy + marker_length, dz - marker_length], [dx + marker_length, dy + marker_length, dz - marker_length], [dx + marker_length, dy, dz - marker_length]
])

# Define the vertices for the prism (example prism with a different shape)
PRISMA_VERTICES = np.float32([
    [dx, dy, dz], [dx, dy + marker_length, dz], [dx + marker_length, dy + marker_length, dz], [dx + marker_length, dy, dz],
    [dx + 0.5*marker_length, dy + 0.5*marker_length, dz - marker_length], [dx + 0.5*marker_length, dy + 0.5*marker_length, dz - marker_length], [dx + 1.5*marker_length, dy + 1.5*marker_length, dz - marker_length], [dx + 1.5*marker_length, dy + 1.5*marker_length, dz - marker_length]
])

# Function to draw a cube on the frame using CUBE_VERTICES
def draw_cube(frame, imgpts):
    imgpts = np.int32(imgpts).reshape(-1, 2)
    
    # Draw ground floor in green
    frame = cv.drawContours(frame, [imgpts[:4]], -1, (0, 255, 0), 3)
    
    # Draw pillars in blue
    for i, j in zip(range(4), range(4, 8)):
        frame = cv.line(frame, tuple(imgpts[i]), tuple(imgpts[j]), (255, 0, 0), 3)
    
    # Draw top layer in red
    frame = cv.drawContours(frame, [imgpts[4:]], -1, (0, 0, 255), 3)
    
    return frame

# Function to draw a prism on the frame using PRISMA_VERTICES
def draw_prism(frame, imgpts):
    imgpts = np.int32(imgpts).reshape(-1, 2)
    
    # Draw ground floor in yellow
    frame = cv.drawContours(frame, [imgpts[:4]], -1, (0, 255, 255), 3)
    
    # Draw pillars in cyan
    for i, j in zip(range(4), range(4, 8)):
        frame = cv.line(frame, tuple(imgpts[i]), tuple(imgpts[j]), (255, 255, 0), 3)
    
    # Draw top layer in magenta
    frame = cv.drawContours(frame, [imgpts[4:]], -1, (255, 0, 255), 3)
    
    return frame

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
            
            # Select the vertices based on the marker ID
            if ids[i] == 25:
                vertices = CUBE_VERTICES
                draw_shape = draw_cube
            elif ids[i] == 30:
                vertices = PRISMA_VERTICES
                draw_shape = draw_prism
            else:
                continue  # Skip drawing if it's not 25 or 30
            
            # Project 3D points to image plane
            imgpts, _ = cv.projectPoints(vertices, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
            
            # Draw the shape (cube or prism)
            frame = draw_shape(frame, imgpts)

    cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
