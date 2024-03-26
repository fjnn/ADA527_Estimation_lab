import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import ast

from Classes import KalmanFilter
from Classes import Recorder


def main():
    # Define Kalman filter parameters
    dt = 0.1  # Time step
    A = np.array([[1, 0], [0, 1]])
    B = np.array([[0, 0]]).T
    C = np.array([[1, 0], [0, 1]])
    x0 = np.array((2, 1), np.float32) # Initial state estimate
    y0 = np.zeros((2, 1), np.float32)
    Q = np.eye(2) * 0.01  # Process noise covariance
    R = np.eye(2) * 1  # Measurement noise covariance
    P = np.eye(2)  # Initial state covariance

    # Initialize Kalman filter
    kf = KalmanFilter(F=A, B=B, H=C, Q=Q, R=R, Z=y0, X=x0, P=P)

    # Load data (replace this with your data loading code)
    cwd = os.getcwd()+'\\Lab\\recorded_data_lab\\'
    df = pd.read_csv(os.path.join(cwd, 'recorded_data.csv'))
    pixels_from_encoder = Recorder.parse_nested_array(df['pixels_from_encoder'])
    pixels_from_cv2 = Recorder.parse_nested_array(df['pixels_from_cv2'])
    time_list = df['Time']


    # Iterate through data and apply Kalman filter
    estimated_positions = []
    df_estimated_positions = pd.DataFrame(columns=['Time', 'estimated_positions'])
    for i in range(len(time_list)):
        # Predict
        kf.predict(pixels_from_encoder[i])
        
        # Update with measurement
        kf.correct(pixels_from_cv2[i])
        
        # Save into df
        df_estimated_positions.loc[i] = {'Time': time_list[i],
                                      'estimated_positions': kf.X}

    # Print or use estimated_positions as required
    # print("Estimated Pixel Positions:", estimated_positions)
    
    df_estimated_positions.to_csv('estimated_data.csv', index=False)
        
if __name__ == "__main__":
    main()