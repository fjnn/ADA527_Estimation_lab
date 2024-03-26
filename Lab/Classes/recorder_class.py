import cv2
import pandas as pd
import time
from datetime import datetime
import numpy as np


class Recorder:

    def __init__(self, cap, output_filename):
        # Get the default frame width and height
        self.frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        self.out = cv2.VideoWriter(output_filename, fourcc, 60.0, (self.frame_width, self.frame_height))

        self.start_time = cv2.getTickCount()

        # Initialize an empty DataFrame
        self.df = pd.DataFrame(columns=['Time', 'pixels_from_encoder', 'pixels_from_cv2'])
        

        
    def record_data(self, pixels_from_encoder, pixels_from_cv2, encoder_readings):
        self.get_elapsed_time()
        self.df.loc[len(self.df)] = {'Time': self.elapsed_time,
                                      'pixels_from_encoder': pixels_from_encoder,
                                      'pixels_from_cv2': pixels_from_cv2,
                                      'encoder_readings': encoder_readings}
       


    def record_video(self, frame):
        

        # Write the frame to the output video file
        self.out.write(frame)

        self.get_elapsed_time()

        # Display the frame
        cv2.imshow('Recording...', frame)


    def save_to_csv(self, csv_file_name):
        self.df.to_csv(csv_file_name, index=False)

    def get_elapsed_time(self):
        # Calculate the elapsed time
        self.elapsed_time = (cv2.getTickCount() - self.start_time) / cv2.getTickFrequency()

    @classmethod
    def parse_nested_array(cls, nested_arrays_list):
        parsed_arrays = []
        if (nested_arrays_list[0][0] == '['):
            for nested_array_str in nested_arrays_list:
                # Remove the brackets and split the string by spaces
                numbers = nested_array_str.strip('[]').split()

                # Convert strings to integers
                numbers = [int(num) for num in numbers]

                # Convert the list to a numpy array
                numpy_array = np.array(numbers)

                parsed_arrays.append(numpy_array)
        
        elif (nested_arrays_list[0][0] == '('):
            for nested_array_str in nested_arrays_list:
                numbers = np.array([int(x) for x in nested_array_str.strip('()').split(',')])
                parsed_arrays.append(numbers)


        else:
            print("None")
            print(type(nested_arrays_list[0][0]))
            exit()
        
        return parsed_arrays
