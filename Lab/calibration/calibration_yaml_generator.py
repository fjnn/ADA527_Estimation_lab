import numpy as np
import cv2
import glob
import yaml
import os

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,8,0)
objp = np.zeros((7*9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:7].T.reshape(-1, 2)

# Scale the object points to 20x20 mm square sizes
objp = objp * 20

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# Locate your calibration pictures
cwd = os.getcwd()
file_path = cwd+'\\Lab\\calibration\\'
images=[]
for file in os.listdir(file_path):
    if file.endswith('.jpg') or file.endswith('.JPG'):
        images.append(os.path.join(file_path, file))
        print(os.path.join(file_path, file))

found = 0

img = cv2.imread(images[0])

for fname in images:
    # Here, 10 can bcde changed to whatever number you like to choose
    img = cv2.imread(fname)  # Capture frame-by-frame
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9, 7), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9, 7), corners2, ret)
        found += 1
        cv2.imshow('img', img)
        cv2.waitKey(500)

print("Number of images used for calibration: ", found)
cv2.destroyAllWindows()

# calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# transform the matrix and distortion coefficients to writable lists
data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}

# and save it to a file
with open(file_path+"calibration_matrix.yaml", "w") as f:
    yaml.dump(data, f)

# done