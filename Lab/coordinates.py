import cv2

# Function to display mouse coordinates
def display_coordinates(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        # Display mouse coordinates in the top-left corner of the frame
        cv2.putText(frame, f'({x}, {y})', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Webcam', frame)

# Open webcam
cap = cv2.VideoCapture(1)

# Create a window
cv2.namedWindow('Webcam')

# Set mouse callback function
cv2.setMouseCallback('Webcam', display_coordinates)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if the frame was successfully captured
    if not ret:
        print("Error: Couldn't capture frame.")
        break

    # Display the frame
    cv2.imshow('Webcam', frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()
