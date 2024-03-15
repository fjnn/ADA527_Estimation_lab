'''
This file is for calibrating the camera-qube positions manually.
The calibration is not perfect but we don't need it to be perfect. Kalman filter should fix the errors later on.
Try to make sure the front face of the qube is fully within the red circle.
'''

import cv2

# Open webcam
cap = cv2.VideoCapture(1)

img_counter = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("failed to grab frame")
        break

    cv2.rectangle(frame, (545, 425), (750, 640), (0, 20, 220), 3)
    cv2.imshow("test", frame)

    k = cv2.waitKey(1)
    # Exit if 'q' is pressed
    if k & 0xFF == ord('q'):
        break

