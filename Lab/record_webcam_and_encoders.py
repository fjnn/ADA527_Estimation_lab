import cv2
from Classes import Qube

def record_encoders():
    qube_object = Qube()
    encoder_angles = qube_object.read_encoders_once()
    qube_object.kinematics(theta=encoder_angles[0], alpha=encoder_angles[1])    


def record_video(output_filename, duration):
    # Open webcam
    cap = cv2.VideoCapture(1)

    # Get the default frame width and height
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    out = cv2.VideoWriter(output_filename, fourcc, 60.0, (frame_width, frame_height))

    start_time = cv2.getTickCount()

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame was successfully captured
        if not ret:
            print("Error: Couldn't capture frame.")
            break

        # Write the frame to the output video file
        out.write(frame)

        # Calculate the elapsed time
        elapsed_time = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()

        # Break the loop if the duration has been reached
        if elapsed_time >= duration:
            break

        # Display the frame
        cv2.imshow('Recording...', frame)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release everything
    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    output_filename = 'output_video2.mp4'
    duration = 30  # Duration in seconds
    record_video(output_filename, duration)
    # output_image = r'C:\Users\gizem\Desktop\output_imagee.jpg'
    # output_image = 'output_imagee.jpg'
    # record_image(output_image)
