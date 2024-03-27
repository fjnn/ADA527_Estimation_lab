import cv2
import numpy as np
import pandas as pd
import os

from Classes import RedRectangle
from Classes import PixelToWorldCoordinates
from Classes import Recorder


cwd = os.getcwd()

video_path = cwd+'\\Lab\\recorded_data_lab\\'
input_video_path = video_path+'output_video_with_encoders.mp4'
cap = cv2.VideoCapture(input_video_path) ## Use cv2.VideoCapture(1) for real camera.

rectangle_detector = RedRectangle()
pixel_capture = PixelToWorldCoordinates(cap=cap, cwd=cwd, calib_file_name='calibration_matrix.yaml')

# Load data (replace this with your data loading code for part-2)
cwd = os.getcwd()+'\\Lab\\recorded_data_lab\\'
df_recorded = pd.read_csv(os.path.join(cwd, 'recorded_data.csv'))
df_estimated = pd.read_csv(os.path.join(cwd, 'estimated_data.csv'))
pixels_from_encoder = Recorder.parse_nested_array(df_recorded['pixels_from_encoder'])
pixels_from_cv2 = Recorder.parse_nested_array(df_recorded['pixels_from_cv2'])
estimated_positions = Recorder.parse_nested_array(df_estimated['estimated_positions'])
time_list = df_estimated['Time']

i = 0
while i<len(time_list):
    # Capture frame from the seected video type
    ret, frame = pixel_capture.cap.read()

    # Undistort the frame
    undistorted_frame = cv2.undistort(frame, pixel_capture.camera_matrix, pixel_capture.dist_coeff)

    # Draw origin and mouse position on the frame
    cv2.circle(undistorted_frame, pixel_capture.origin, 5, (0, 0, 255), -1)
    cv2.circle(undistorted_frame, pixels_from_cv2[i], 5, (0, 255, 0), -1)
    cv2.circle(undistorted_frame, pixels_from_encoder[i]-[-80, 0], 5, (255, 0, 0), -1)
    cv2.circle(undistorted_frame, estimated_positions[i][:2], 5, (255, 255, 0), -1)

    cv2.putText(undistorted_frame, 'Origin', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(undistorted_frame, 'Camera', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(undistorted_frame, 'System model', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(undistorted_frame, 'Estimated', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # Display the frame
    cv2.imshow('Frame', undistorted_frame)
    i += 1

    # Exit if 'q' is pressed
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()