# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

import cv2
import numpy as np
import yaml
import os
import spatialmath.base as base


# Locate your camera_calibration.yaml file
cwd = os.getcwd()+'\\Lab\\calibration\\'

with open(os.path.join(cwd, 'calibration_matrix.yaml'), 'r') as stream:
    calibration_data = yaml.safe_load(stream)

# print(calibration_data)
camera_matrix = np.array(calibration_data['camera_matrix'])
dist_coeff = np.array(calibration_data['dist_coeff'])

com_world_coord = np.array([21.8988748,  62.00489438, 87.29877163])

img = cv2.imread(cwd+'output_image_qube.jpg')
## TODO: This needs adjustments all the time... Find out why.
origin_offset_y = 175
origin_offset_x = 40

# Define origin point
origin = (int(camera_matrix[0, 2])+origin_offset_x, int(camera_matrix[1, 2])+origin_offset_y)

# Perform projection
image_coords, _ = cv2.projectPoints(com_world_coord, np.eye(3), np.zeros(3), camera_matrix, dist_coeff)

# Convert floating point pixel coordinates to integers
image_coords = np.round(image_coords).astype(int)

# Print the result
print("World coordinates:", com_world_coord)
print("Image coordinates:", image_coords)
removed_offset = image_coords - [origin_offset_x, origin_offset_y]

# Draw points on the image (optional)
for i, coord in enumerate(removed_offset):
    cv2.circle(img, tuple(coord.ravel()), 5, (0, 255, 0), -1)

# Show the image with drawn points (optional)
cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()