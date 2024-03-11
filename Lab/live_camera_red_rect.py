import cv2
import numpy as np

def detect_red_stick(frame):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of red color in HSV
    hsv_vals = [[ 161 , 163 , 79 ], [ 181 , 263 , 179 ]]
    lower_red = np.array(hsv_vals[0])
    upper_red = np.array(hsv_vals[1])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through the contours
    for contour in contours:

        ## 1) Only vertical rectangle
        # # Get the bounding rectangle
        # x, y, w, h = cv2.boundingRect(contour)
        # aspect_ratio = w / float(h)

        # # Filter out rectangles that are not vertical enough
        # if 0.01 < aspect_ratio and h > 150:
        #     # Draw the bounding rectangle around the stick
        #     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        #     print("w:", w, "    h:", h)

        ## 2) Parallelogram -- thats what we need
        # Fit a rotated rectangle to the contour
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # Draw the rotated rectangle
        cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

    return frame

def main():
    # Open webcam
    cap = cv2.VideoCapture(1)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame was successfully captured
        if not ret:
            print("Error: Couldn't capture frame.")
            break

        # Detect the red stick
        detected_frame = detect_red_stick(frame)

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
