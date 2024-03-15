import cv2
import numpy as np
from Classes import RedRectangle 


def main():
    # Open webcam
    cap = cv2.VideoCapture(1)
    rectangle_detector = RedRectangle()

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame was successfully captured
        if not ret:
            print("Error: Couldn't capture frame.")
            break

        # Detect the red stick
        detected_frame = rectangle_detector.detect_red_stick(frame)

        # Display the frame
        cv2.imshow('Red Stick Detection', detected_frame)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
