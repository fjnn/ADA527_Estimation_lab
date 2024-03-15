import cv2

def show_camera():
    # Open the first camera device
    cap = cv2.VideoCapture(1)

    # Check if the camera is opened successfully
    if not cap.isOpened():
        print("Error: Couldn't open the camera.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame was successfully captured
        if not ret:
            print("Error: Couldn't capture frame.")
            break

        # Display the frame
        cv2.imshow('USB Camera', frame)

        # Wait for 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    show_camera()