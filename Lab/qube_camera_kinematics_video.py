# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

import cv2
import numpy as np
import yaml
import os
import spatialmath.base as base
from Classes import Qube
from time import sleep
from math import degrees


cwd = os.getcwd()
# Locate your camera_calibration.yaml file
cwd_calib = cwd+'\\Lab\\calibration\\'

# Locate your recorded video
video_path = cwd+'\\Lab\\recorded_data\\'
input_video_path = video_path+'output_video3.mp4'
cap = cv2.VideoCapture(input_video_path) ## Use cv2.VideoCapture(1) for real camera.


with open(os.path.join(cwd_calib, 'calibration_matrix.yaml'), 'r') as stream:
    calibration_data = yaml.safe_load(stream)

# print(calibration_data)
camera_matrix = np.array(calibration_data['camera_matrix'])
dist_coeff = np.array(calibration_data['dist_coeff'])

# com_world_coord = np.array([21.8988748,  62.00489438, 87.29877163])

img = cv2.imread(cwd_calib+'output_image_qube.jpg')
img_without_circles = img.copy()
origin_offset_y = 275
origin_offset_x = 10

init_com_x = 80
init_com_y = 160

# Define origin point
origin = (int(camera_matrix[0, 2])+origin_offset_x, int(camera_matrix[1, 2])+origin_offset_y)
init_com = (int(camera_matrix[0, 2])+init_com_x, int(camera_matrix[1, 2])+init_com_y)
cv2.circle(img, origin, 5, (0, 0, 255), -1)

qube_object = Qube()

key = cv2.waitKey(1) & 0xFF  # Mask with 0xFF to get the last 8 bits

while True:
    encoder_readings = qube_object.read_encoders_once()
    com_encoder_world_frame = qube_object.kinematics(encoder_readings[0], encoder_readings[1])
    com_encoder_camera_frame = qube_object.qube_to_camera(com_encoder_world_frame)
    # print("com_encoder: ", com_encoder_camera_frame, "angles: ", degrees(encoder_readings[0]), degrees(encoder_readings[1]))

    # Perform projection
    image_coords, _ = cv2.projectPoints(com_encoder_camera_frame, np.eye(3), np.zeros(3), camera_matrix, dist_coeff)

    # Convert floating point pixel coordinates to integers
    image_coords = np.round(image_coords).astype(int)

    # Print the result
    removed_offset = image_coords - [init_com_x, init_com_y]
    print("removed coordinates:", removed_offset[0], "World coordinates:", com_encoder_camera_frame)

    # Draw points on the image (optional)
    for i, coord in enumerate(removed_offset):
        cv2.circle(img, tuple(coord.ravel()), 5, (0, 255, 0), -1)

    # Show the image with drawn points (optional)
    cv2.imshow('Image', img)

    # Exit if 'q' is pressed
    key = cv2.waitKey(1) & 0xFF 
    if key == ord('q'):
        break

        # Exit if 's' is pressed
    elif key == ord('s'):
        init_com_x = image_coords[0,0,0] - init_com[0]
        init_com_y = image_coords[0,0,1] - init_com[1]
        print("Registered:  ", init_com_x, "---", init_com_x)

print("KeyboardInterrupt received. Exiting...")

qube_object.close_all()
cv2.destroyAllWindows()