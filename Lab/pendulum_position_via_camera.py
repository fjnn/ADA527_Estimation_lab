import cv2
import numpy as np
import yaml
import os

from Classes import RedRectangle 
from Classes import PixelToWorldCoordinates


cwd = os.getcwd()

video_path = cwd+'\\Lab\\recorded_data\\'
input_video_path = video_path+'output_video2.mp4'
cap = cv2.VideoCapture(input_video_path) ## Use cv2.VideoCapture(1) for real camera.

rectangle_detector = RedRectangle()
pixel_capture = PixelToWorldCoordinates(cap=cap, cwd=cwd, calib_file_name='calibration_matrix.yaml')

while True:
    # Capture frame from webcam
    ret, frame = pixel_capture.cap.read()

    # Undistort the frame
    undistorted_frame = cv2.undistort(frame, pixel_capture.camera_matrix, pixel_capture.dist_coeff)

    # Draw origin and mouse position on the frame
    cv2.circle(undistorted_frame, pixel_capture.origin, 5, (0, 0, 255), -1)

    # Detect the red stick
    detected_frame = rectangle_detector.detect_red_stick(undistorted_frame)
    com_pixels = rectangle_detector.get_com_pixels()
    com_coordinates = pixel_capture.convert_pixels_to_world_coordinates(undistorted_frame, com_pixels)
    # print("CoM world coordinates:", com_coordinates)
    face_width_in_frame = rectangle_detector.get_stick_width_in_pixels()
    try:
        distance = rectangle_detector.distance_finder(face_width_in_frame=face_width_in_frame)
    except ZeroDivisionError:
        distance = rectangle_detector.measured_distance

    # print("distance:   ",distance, "     width:  ", face_width_in_frame)
    # print(f'Distance: {distance:.2f}')
    # print(f'Width: {face_width_in_frame:.2f}')
    cv2.putText(undistorted_frame, f'World Coords: ({com_coordinates[0]*distance:.2f}, {com_coordinates[1]*distance:.2f}, {com_coordinates[2]:.2f})', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('Frame', detected_frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows() 