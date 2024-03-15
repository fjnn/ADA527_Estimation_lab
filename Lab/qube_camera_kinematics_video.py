# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

import cv2
import numpy as np
import yaml
import os
import spatialmath.base as base
from Classes import Qube
from time import sleep
from math import degrees

from Classes import RedRectangle 
from Classes import PixelToWorldCoordinates
from Classes import Recorder

def com_from_video_frame(frame, rectangle_detector):
    # Detect the red stick
    detected_frame = rectangle_detector.detect_red_stick(frame)
    com_pixels = rectangle_detector.get_com_pixels()
    com_coordinates = pixel_capture.convert_pixels_to_world_coordinates(frame, com_pixels)
    # print("CoM world coordinates:", com_coordinates)
    face_width_in_frame = rectangle_detector.get_stick_width_in_pixels()
    face_height_in_frame = rectangle_detector.get_stick_height_in_pixels()
    try:
        distance = rectangle_detector.distance_finder(face_width_in_frame=face_width_in_frame)
    except ZeroDivisionError:
        distance = rectangle_detector.measured_distance

    # print("distance:   ",distance, "     width:  ", face_width_in_frame, "     height:  ", face_height_in_frame)
    # print(f'Distance: {distance:.2f}')
    # print(f'Width: {face_width_in_frame:.2f}')
    cv2.putText(frame, f'World Coords: ({com_coordinates[0]*distance:.2f}, {com_coordinates[1]*distance:.2f}, {com_coordinates[2]:.2f})', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return detected_frame, com_coordinates, com_pixels


def com_from_encoders(frame, qube_object):
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
        cv2.circle(frame, tuple(coord.ravel()), 5, (0, 255, 0), -1)

    return image_coords, removed_offset


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


origin_offset_y = 275
origin_offset_x = 10

init_com_x = -80
init_com_y = 0

# cap = cv2.VideoCapture(input_video_path) ## Use cv2.VideoCapture(1) for real camera.
cap = cv2.VideoCapture(1) ## Use cv2.VideoCapture(1) for real camera.

rectangle_detector = RedRectangle()
pixel_capture = PixelToWorldCoordinates(cap=cap, cwd=cwd, calib_file_name='calibration_matrix.yaml')
qube_object = Qube()
recorder_object = Recorder(cap, output_filename='output_video_with_encoders.mp4')
record_duration = 5


# Define origin point
origin = (int(camera_matrix[0, 2])+origin_offset_x, int(camera_matrix[1, 2])+origin_offset_y)
init_com = (int(camera_matrix[0, 2])+init_com_x, int(camera_matrix[1, 2])+init_com_y)

while True:

    # Capture frame from the seected video type
    ret, frame = pixel_capture.cap.read()

    # Undistort the frame
    undistorted_frame = cv2.undistort(frame, pixel_capture.camera_matrix, pixel_capture.dist_coeff)

    # Draw origin and mouse position on the frame
    cv2.circle(undistorted_frame, pixel_capture.origin, 5, (0, 0, 255), -1)

    com_pixels_from_encoder, com_coordinates_from_encoder = com_from_encoders(undistorted_frame, qube_object)
    detected_frame, com_coordinates_from_video, com_pixels_from_video = com_from_video_frame(undistorted_frame, rectangle_detector)

    # Record video and data
    recorder_object.record_data(qube_object, com_pixels_from_encoder, com_pixels_from_video)
    recorder_object.record_video(frame)
    print(recorder_object.elapsed_time)

    if recorder_object.elapsed_time >= record_duration:
        break

    # Display the frame
    # cv2.imshow('Frame', detected_frame)

    # Exit if 'q' is pressed
    key = cv2.waitKey(1) & 0xFF 
    if key == ord('q'):
        break

    # Exit if 's' is pressed
    elif key == ord('s'):
        init_com_x = com_pixels_from_encoder[0,0,0] - init_com[0]
        init_com_y = com_pixels_from_encoder[0,0,1] - init_com[1]
        print("Registered:  ", init_com_x, "---", init_com_x)

print("KeyboardInterrupt received. Exiting...")

cap.release()
recorder_object.out.release()
qube_object.close_all()
cv2.destroyAllWindows()

