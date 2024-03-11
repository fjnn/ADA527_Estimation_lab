# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

import cv2
import numpy as np
import yaml
import os

# Locate your camera_calibration.yaml file
cwd = os.getcwd()

with open(os.path.join(cwd, 'camera_calibration.yaml'), 'r') as stream:
    calibration_data = yaml.safe_load(stream)

# print(calibration_data)
camera_matrix = np.array(calibration_data['camera_matrix'])
dist_coeff = np.array(calibration_data['dist_coeff'])


cap = cv2.VideoCapture(1)
origin_offset_y = 275
origin_offset_x = 10

# Define origin point
origin = (int(camera_matrix[0, 2])+origin_offset_x, int(camera_matrix[1, 2])+origin_offset_y)

# Mouse callback function
mouse_x, mouse_y = None, None

def mouse_move(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y
# Set mouse callback
cv2.namedWindow('Frame')
cv2.setMouseCallback('Frame', mouse_move)

while True:
    # Capture frame from webcam
    ret, frame = cap.read()

    # Undistort the frame
    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeff)

    # Get mouse position (if available)
    if mouse_x is not None and mouse_y is not None:
        # Convert mouse position to world coordinates
        mouse_position = np.array([[mouse_x-origin_offset_x, mouse_y-origin_offset_y]], dtype=np.float32)
        mouse_position = cv2.undistortPoints(mouse_position, camera_matrix, dist_coeff)
        mouse_world_x, mouse_world_y = mouse_position[0, 0]

        # Draw origin and mouse position on the frame
        cv2.circle(undistorted_frame, origin, 5, (0, 0, 255), -1)
        cv2.circle(undistorted_frame, (mouse_x, mouse_y), 5, (0, 255, 0), -1)
        cv2.putText(undistorted_frame, f'World Coords: ({mouse_world_x:.2f}, {mouse_world_y:.2f})', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('Frame', undistorted_frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()