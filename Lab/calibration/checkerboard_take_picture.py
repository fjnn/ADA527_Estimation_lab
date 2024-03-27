import cv2
import os

# Locate your camera_calibration.yaml file
cwd = os.getcwd()

# Initialize the webcam
cap = cv2.VideoCapture(1)

# Flag to keep track of when to capture an image
capture_image = False

# Counter for naming the captured images
image_counter = 0

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    # Display the frame
    cv2.imshow('Webcam', frame)

    # Check for the space bar key press
    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):
        capture_image = True

    # Capture an image if the flag is set
    if capture_image:
        # Construct the image file name
        image_name = f'captured_image_{image_counter}.jpg'

        # Save the image
        cv2.imwrite(image_name, frame)
        print(f'Image saved as {image_name}')

        # Increment the image counter
        image_counter += 1

        # Reset the capture flag
        capture_image = False

    # Exit the loop if 'q' is pressed
    if key == ord('q'):
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()