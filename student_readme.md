# ADA527 Lab-2: Qube and Kalman filter

So, you are familiar with the Qube system thanks to Lab-1. It is basically a 2 DoF inverted pendulum or also called "rotary pendulum". You can control the theta and alpha angles to, let's say, balance the pendulum.



What if you want to know about where the center point of the red rod is while you are doing some motions? How can you calculate it? Should you use a camera and basically implement a *red rod tracker* algorithm and find the middle point? Or should you use the encoder readings and do some kinematics to calculate the center of the rod?



Well, we know that both methods have some limitations:

1.  When you use a camera, you have to calibrate it so well that you would have a reliable *red rod tracker* algorithm. Moreover, you must keep in mind that any obstacles between the camera and the Qube is your enemy. Also, you better to spend a lot of money to have a camera with exceptionally good resolution and FPS.

2. When you use the system model, you are limited by the encoder resolution and how well the system model is defined. Luckily, the encoders are pretty good in this overly priced device. However, depending on your system model, the pose of center point of the rod would be less accurate. Also, any inaccuracies on the encoders would be multiplied by the link lenghts and would cause even bigger errors on the position of the center of the rod.
   
   
   

We know by now that it is better to *fuse* the things that to increase reliability of the estimation. Kalman filter is the best for this purpose. Therefore, this lab is about fusing the system model of the Qube and observations via camera to estimate the position of the center of the rode more reliably.



For that, we need to express things in this format.

<img src="file:///C:/Users/gizem/AppData/Roaming/marktext/images/2024-03-27-12-33-35-image.png" title="" alt="" data-align="center">

## Deciding the state and observation matrices

As you know, **state matrix** is a matrix form of your **system model**. The system model is a mathematical expression that relates your system states to an output entity. The system states are alpha, theta, and their derivatives and the output is the pose of the center of rod in this setup. You can use various methods to *generate* such a mathematical expression.

It is the same for the **observation matrix**. There, you would write down a mathematical expression that links your states and whatever entity that you measure/observe/estimate.

### a. A bit background/brainstorming

One way is just to use the state equations that you derived in Lab-1. Something like this:

<img title="" src="file:///C:/Users/gizem/AppData/Roaming/marktext/images/2024-03-27-12-26-47-image.png" alt="" data-align="center" width="467">

And if you used the encoder readings as your *observation*,  then the observer equations would be like this:

<img title="" src="file:///C:/Users/gizem/AppData/Roaming/marktext/images/2024-03-27-12-27-02-image.png" alt="" data-align="center" width="305">

However, we would like to elaborate with the camera readings in this lab. 



At this point, you should take a decision. Are you planning to **observe the theta and alpha angles** or the **pixel positions** via your camera. 

1. In the first option, you can extract the feature, let's say a red rectangle, in your frame, then calculate the angle. For alpha angle, it would look like this:

![](C:\Users\gizem\AppData\Roaming\marktext\images\2024-03-27-13-28-13-image.png)

2. Or we can extract pixel positions, and they can be our observation. Then, our observer equations would rather be like in this format:

<img src="file:///C:/Users/gizem/AppData/Roaming/marktext/images/2024-03-27-12-40-01-image.png" title="" alt="" data-align="center">

    where x and y are the pixel positions on our recorded image/video and shown as the blue circle in the picture above. 



**Disclaimer**: The first method might given you better estimation results but due to "various limitations", we will go with the second method.

### b. Decision

Since we decided to observe pixel positions the steps will be like this:

1. Obtain the position of the center of the red rod using robot kinematics. This would give us a pose, let's say center-of-mass of the rod, in the Qube frame: $^B\xi_{com}$  

2. Transform $^B\xi_{com}$ into world frame: $^W\xi_{com} =  ^WR_B \cdot ^B\xi_{com}$



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

1. qube_camera_kinematics.py
   This is for

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

Kalman filter implementation.
