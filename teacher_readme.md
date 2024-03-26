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

1. qube_camera_kinematics.py
   This is for

## Files

live_camera_red_rect.py: Detects pendulum as a red rectangle.

record_webcam.py: Record a video via camera

## TODO:

Balance_pendulum.py and camera record in one file and register CoM pose and encoder readings at the same time. DONE

Kalman filter implementation.
