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
    A = np.array([[1, 0, dt, 0], [0, 1, 0 ,dt], [0, 0, 1, 0], [0, 0, 0, 1]])
    B = np.array([[0, 0, 0, 0]]).T
    C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
    x0 = np.zeros((4, 1), np.float32) # Initial state estimate
    y0 = np.zeros((2, 1), np.float32)
    Q = np.eye(4) * 0.1  # Process noise covariance
    R = np.eye(2) * 0.01  # Measurement noise covariance
    P = np.eye(4)  # Initial state covariance

    # Initialize Kalman filter
    kf = KalmanFilter(F=A, B=B, H=C, Q=Q, R=R, Z=y0, X=x0, P=P)

    # Load data (replace this with your data loading code for part-2)
    cwd = os.getcwd()+'\\Lab\\recorded_data_lab\\'
    df = pd.read_csv(os.path.join(cwd, 'recorded_data.csv'))
    pixels_from_encoder = Recorder.parse_nested_array(df['pixels_from_encoder'])
    pixels_from_cv2 = Recorder.parse_nested_array(df['pixels_from_cv2'])
    time_list = df['Time']

    pixels_from_encoder[0] = np.array([pixels_from_encoder[0][0], pixels_from_encoder[0][1], 0, 0])


    # Iterate through data and apply Kalman filter
    df_estimated_positions = pd.DataFrame(columns=['Time', 'estimated_positions'])
    for i in range(1, len(time_list)):
        # Predict
        kf.predict(pixels_from_encoder[i-1])
        
        # Update with measurement
        kf.correct(pixels_from_cv2[i])
        
        # Save into df
        df_estimated_positions.loc[i] = {'Time': time_list[i],
                                      'estimated_positions': kf.X.astype(int)}
        
        pixels_from_encoder[i] = np.array([pixels_from_encoder[i][0], pixels_from_encoder[i][1], 
                                           kf.X.astype(int)[0], kf.X.astype(int)[1]])

        

    # Print or use estimated_positions as required
    # print("Estimated Pixel Positions:", estimated_positions)
    
    df_estimated_positions.to_csv(os.path.join(cwd, 'estimated_data.csv'), index=False)
    print("done")
        
if __name__ == "__main__":
    main()