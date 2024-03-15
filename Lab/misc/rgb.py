import cv2

# Global variables to store mouse position and pixel values
mouse_x, mouse_y = 0, 0
rgb_values = None

# Function to update mouse position
def update_mouse_position(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y

# Function to update pixel RGB values
def update_pixel_values(frame):
    global rgb_values
    if frame is not None:
        b, g, r = frame[mouse_y, mouse_x]
        rgb_values = f"RGB: ({r}, {g}, {b})"

# Open webcam
cap = cv2.VideoCapture(1)

# Create a window
cv2.namedWindow('Webcam')
cv2.setMouseCallback('Webcam', update_mouse_position)

# Timer to keep the latest text update for 2 seconds
text_timer = 0

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if the frame was successfully captured
    if not ret:
        print("Error: Couldn't capture frame.")
        break

    # Update pixel RGB values
    update_pixel_values(frame)

    # Display the pixel values in the top-left corner of the frame
    if rgb_values is not None:
        cv2.putText(frame, rgb_values, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        text_timer += 1
        if text_timer >= 100:  # Display the text for 2 seconds (100 frames at 50 FPS)
            rgb_values = None
            text_timer = 0

    # Display the frame
    cv2.imshow('Webcam', frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()
