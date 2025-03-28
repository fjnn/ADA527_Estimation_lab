# ADA527_Estimation_lab

# Estimation Lab

Everything is under C:\Users\gizem\Documents\Git-workspace\ADA527_Estimation_lab>

~~C:\Users\gizem\OneDrive - Høgskulen på Vestlandet\HVL\Teaching\Courses\ADA527\ADA527-labs\ADA527-labs\4-State Estimation\Proposition number 3 - Qube + python\Lab>~~

## Important!

Please do not use HVL Onedrive as your project location in this lab. Choose a path (preferably short) and contains to space character in the path.

## Camera calibration

Files are under /Lab/calibration/ folder.

1. Place things correctly on the “workspace board”:
   Make sure the camera and Qube placed correctly: **manual_calib.py**
2. Generate calibration images:
   Take some checkerboard pictures. The script is designed for 7x9 checkerboard with 20 mm squares. Try to take around 10 pictures with different angles and distances: **checkerboard_take_picture.py**
3. Generate calibration YAML file:
   Locate those picture and decide a location for a YAML file that is going to hold the calibration parameters. The locations might be needed to modify in the code. When you do the necessary changes, run: **calibration_camera_generator.py**
4. Test if your coordinates are translated correctly:
   To make sure that you did all the calibration steps correctly, you should see (0,0) coordinates on the left top corner when you move your mouse on the Qube origin in this script: **pixel_to_world_coordinates.py** The red dot visualize the Qube origin.

## Actual robotic part of the lab starts here

Now we have the world and camera coordinates into the pocket, we can proceed with the robotics calculations. The aim is to estimate x and y coordinates of the middle point (I referred as center of mass in the code (CoM)).

### The point is

We calculate (x, y) coordinates of the CoM of the pendulum using two different methods.

1. system model
2. image processing

And then, use them in the Kalman filter.

Finally, we compare the (x, y) coordinated found by the KF (this is our observation vector (aka **y** vector) with the encoder readings to evaluate the performance of our KF.

### Part 1 Using recorded data

The purpose of this step is to setup your Kalman filter and ensure that it is working on a system that you know what the output should be (somehow). In part-2, you will record your own data and use the KF that you configured in part-1.

Here we you have the data under **recorded_data_lab ** folder. There is one mp4 and one csv file. The csv file contains the pixel positions from both the system model and the image processing.

1. qube_camera_kinematics_video.py
   This is for recording data. The file will generate **output_video_with_encoders.mp4** and **recorded_data_csv** under **recorded_data_lab** folder. In default, the data is generated from the camera recordings, but it is possible to use a pre-recorded data by just changing line 67 (cap = cv2.VideoCapture(input_video_path) ## Use cv2.VideoCapture(1) for real camera.) and name your pre-recorded data as **input_video.mp4**.
2. Currently, we are using **KF-qube-using-pixels.py** but later on I want to improve my system model and use **KF-qube-using-encoders.py**.

### Part 2 Record your data and compare

So, the given data was from a simple and slow motion. Now, you are supposed to record it on your own.

1. **qube_camera_kinematics_video.py ** is the script that you will use for recording. It as you run it, it will start recording one video (mp4) and one CSV file that has 3 columns: time, pixel coordinates from image processing, pixel coordinates from system model.

## Files

live_camera_red_rect.py: Detects pendulum as a red rectangle.

record_webcam.py: Record a video via camera

## TODO:

Balance_pendulum.py and camera record in one file and register CoM pose and encoder readings at the same time. DONE

I forgot recording encoder values...

TODO: register torque values?

TODO: for qube_camera_kinematics.

Kalman filter implementation on better state transition model.
