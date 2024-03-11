import cv2
import numpy as np
import yaml
import os

# Locate your camera_calibration.yaml file
cwd = os.getcwd()
calibration_path = cwd+'\\Lab\\calibration\\'

with open(os.path.join(calibration_path, 'camera_calibration.yaml'), 'r') as stream:
    calibration_data = yaml.safe_load(stream)

# print(calibration_data)
camera_matrix = np.array(calibration_data['camera_matrix'])
dist_coeff = np.array(calibration_data['dist_coeff'])

video_path = cwd+'\\Lab\\recorded_data\\'
input_video_path = video_path+'output_video.mp4'
cap = cv2.VideoCapture(input_video_path)


origin_offset_y = 275
origin_offset_x = 10

# Define origin point
origin = (int(camera_matrix[0, 2])+origin_offset_x, int(camera_matrix[1, 2])+origin_offset_y)

# Center of Mass of the pendulum
com_x, com_y = None, None

while True:
    # Capture frame from webcam
    ret, frame = cap.read()

    # Undistort the frame
    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeff)
    
    # Draw origin and mouse position on the frame
    cv2.circle(undistorted_frame, origin, 5, (0, 0, 255), -1)
        # cv2.circle(undistorted_frame, (mouse_x, mouse_y), 5, (0, 255, 0), -1)
        # cv2.putText(undistorted_frame, f'World Coords: ({mouse_world_x:.2f}, {mouse_world_y:.2f})', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('Frame', undistorted_frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()