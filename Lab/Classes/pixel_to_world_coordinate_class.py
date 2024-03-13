# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

import cv2
import numpy as np
import yaml
import os

class PixelToWorldCoordinates:
    
    def __init__(self, cap, cwd=None, calib_file_name='calibration_matrix.yaml') -> None:
        '''
        Parameters:
        -----------
        calib_file_path: Calibration yaml file path. Keep empty if you won't use abother calibration yaml than default.
        
        calib_file_name: Calibration yaml file name. Keep empty if you won't use abother calibration yaml than default.
        '''
        # Locate your camera_calibration.yaml file
        if cwd == None:
            self.cwd = os.getcwd()
        else:
            self.cwd = cwd
        self.calib_file = self.cwd+'\\Lab\\calibration\\'+calib_file_name

        with open(os.path.join(cwd, self.calib_file), 'r') as stream:
            self.calibration_data = yaml.safe_load(stream)

        # print(calibration_data)
        self.camera_matrix = np.array(self.calibration_data['camera_matrix'])
        self.dist_coeff = np.array(self.calibration_data['dist_coeff'])

        self.cap = cap

        # Where is the origin with respect to the camera center
        self.origin_offset_y = 275
        self.origin_offset_x = 10

        # Define origin point
        self.origin = (int(self.camera_matrix[0, 2])+self.origin_offset_x, int(self.camera_matrix[1, 2])+self.origin_offset_y)

        # Center of Mass of the pendulum
        self.com_x, self.com_y = None, None


    def convert_pixels_to_world_coordinates(self, frame, pixels):
        # Convert pixels to world coordinates
        pixel_position = np.array([[pixels[0]-self.origin_offset_x, pixels[1]-self.origin_offset_y]], dtype=np.float32)
        pixel_position = cv2.undistortPoints(pixel_position, self.camera_matrix, self.dist_coeff)
        pixel_world_x, pixel_world_y = pixel_position[0, 0]
        return pixel_position

        # Draw origin and mouse position on the frame
        cv2.circle(undistorted_frame, self.origin, 5, (0, 0, 255), -1)
        cv2.circle(undistorted_frame, (pixel_world_x, pixel_world_y), 5, (0, 255, 0), -1)
        cv2.putText(undistorted_frame, f'World Coords: ({pixel_world_x:.2f}, {pixel_world_y:.2f})', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)