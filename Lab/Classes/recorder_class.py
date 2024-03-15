import cv2


class Recorder:

    def __init__(self, cap, output_filename):
        # Get the default frame width and height
        self.frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        self.out = cv2.VideoWriter(output_filename, fourcc, 60.0, (self.frame_width, self.frame_height))

        self.start_time = cv2.getTickCount()

        
    def record_data(self, qube_object, pixels_from_encoder, pixels_from_cv2):
        # encoder_angles = qube_object.read_encoders_once()
        # qube_object.kinematics(theta=encoder_angles[0], alpha=encoder_angles[1])    
        pass


    def record_video(self, frame):
        

        # Write the frame to the output video file
        self.out.write(frame)

        # Calculate the elapsed time
        self.elapsed_time = (cv2.getTickCount() - self.start_time) / cv2.getTickFrequency()

        # Display the frame
        cv2.imshow('Recording...', frame)




    if __name__ == "__main__":
        output_filename = 'output_video2.mp4'
        duration = 30  # Duration in seconds
        record_video(output_filename, duration)
        # output_image = r'C:\Users\gizem\Desktop\output_imagee.jpg'
        # output_image = 'output_imagee.jpg'
        # record_image(output_image)
