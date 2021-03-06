# Zephyrus - Drone System for Window Detection and Autonomous Control.
This software repository contains the software frameworks and source code to the various algorithms that collectively make the system exhibit an ideal autonomous behaviour and window detection algorithm.

![3D prototype](/images/3dmod.PNG)

The round1:phase1 submission our team made describes a summarized working of our drone. This repository readme aims to provide a deeper insight to our system.

Even though the Round1:Phase1 submission wasnt targeted to a simulation of the drone we actually decided to run a simulation consisting of a dummy drone to test out the various software algorithms on an almost practical environment, however the physical dimensions of this simulated drone isnt the same as our Proposed *Hexacopter* design. 

*The sole aim of this simulation is to verify the working of the Algorithm and the Logic, The submission to Round1:Phase2 will contain a more accurate representation of our actual proposed model with improved software*


**Note : This project is in active development and hence the repository is subject to change however, the source code of algorithms proposed in Phase1 submission will be kept intact** 

## The Depth Perception and implementation of Simultaneous Localization and Mapping for Fully Autonomous Control : 
Before we move into window detection we need to tackle a problem, the drone cannot differentiate between something resembling a window from an actual window intended for the drone to pass through and hence lead to false detections. This can be avoided if the drone can see the depth in addition to the 2-Dimensional image and transform the image into a 3-Dimensional image to analyze the shape and the dimentions of the window and allow the algorithm to only detect windows which fit the dimensions described by the competition guidelines.

In addition to verifying detection process, the 3D perception system also allows the drone to learn the working environment and create a detailed 3D map of the environment. The robot can use its odometry data to localize itself in this map and move itself in accordance to the features of this environment. This process is called **Simultaneous Localization and Mapping(SLAM) algorithm**, this is one of the most vital and versatile algorithms used for Autonomous Control.

In order in execute SLAM or any form of 3D perception task, sensors such as Lidar or Stereo/RGBD Cameras are vital. Unfortunately these sensors are relatively very expensive and hence is often overlooked in projects of this scale. But this is where use a standard 2D image from an onboard camera for an **image preprocessing technique** which involves detection of *feature points* which are tracked over a set of frames of cameras to estimate the *relative depth* of these points.

These estimated relative depth values are highly inaccurate but as the number of frames over which a certain set of feature points are tracked increases the accuracy of this *depth estimation* process significantly improves.

The movement of the camera is estimated as well by comparing the next frame to the previous frame and using a common computer vision technique of Optical Flow.

This estimated motion of the camera is coupled with the estimated depth of feature points to develop a map which progresses as the camera visits newer feature points.

This gives us a final map where the feature points are mapped in a datastructure called PointCloud, the drone can access these at any time for Autonomous algorithms such as 3D A* Pathfinding.

*Feature Point detection and initial depth estimationn at the start of SLAM process:*
![feature points and genetrated SLAM map at the start of the mapping process](/images/SlamStart.png)

*Feature Point Tracking and depth Map at the End of SLAM process:*
![feature points and genetrated SLAM map at the end of the mapping process](/images/SlamPost.png)



## Window Detection Algorithm : 
The detection of the window the drones transverse through is generally done by means of a deep learning algorithm. Our system uses a lot more efficient algorithm for detection of the windows at a notable frame rate.

The detection process in our system works on **Snakegate Algorithm**. we start with the raw image that comes from the camera as the input, then choose a random pixel in the frame for a certain number of iterations. Every iteration, if the randomly chosen pixel matches the color of the gate specified in the competition guidelines, the algorithm proceeds searching the pixels above, below, left and right and checks if they too match the specified color. The search is stopped if the length of pixels from the starting pixel is under a set threshold(not wide enough for a window frame, probably false detection).

if the length of search exceeds the threshold length, we can confirm the pixel is part of the window frame. Now we check specificaly for pixels that dont have matching pixel colors in either one of the horizontal search and either one of the vertical search, we confirm this pixel to be a corner of the frame

iterating over the image for a certain tiimes, we eventually find the pixel coordinates for all 4 corners of the window

*Snakegate window detection on an Image with window in a simulation:*
![feature points and genetrated SLAM map at the end of the mapping process](/images/snakegateFrame.png)

## Control System and Algorithm Design : 
The control system of the Drone aims to control the states of Roll, Pitch, Yaw, thrust and the inputs received from image processing and depth perception system that are the image center and the window detection center.

The algorithm initially begins by acquiring the current states of Roll, Pitch, Yaw from the IMU to control the orientation and hence the movement of the drone in X,Y,Z direction. This movement in X,Y,Z direction is controlled again by controlling the velocities in X,Y,Z directions determined by the change in X,Y,Z values returned by the IMU.

The Control is established by using standard PID controller but selecting the most ideal Setpoint for the inputs in the PID controller is the determining factor of how the drone behaves in response to any change in input and/or Setpoint. The setpoint determination is majorly dominated by the relationship established between the center of the image frame and the window detection.

As long as no window gets detected, the drone transverses a zig zag pattern with scanning yaw in a Raster Scan approack by stepping the thrust. Since the competition guidelines specify the aisle to be a straight one, we will almost every time detect the window in first scan given the wide frame of the FPV drone camera we use in our hardware.

The process begins by computing an Error vector between the image center at any given time and the center of the detected window. This parameter defines our Cost Function.

![feature points and genetrated SLAM map at the end of the mapping process](/images/ErrorVectorEqn.png)

Once the Error vector is determined, the angular parameter of this vector, theta, represents the angle of the point with respect to the horizontal axis in the Drones frame and the X axis of the image frame. This angular parameter is used to determine the values the drone needs to move in Roll(Yaxis), Pitch(Xaxis) and Thrust(Zaxis). 

The Thrust or the vertical velocity is determined by using the vertical component of the vector and similarly the Roll or the Y velocity is determined by using the horizontal component of the vector. Apart from these, we need to set the forward velocity or the pitch setpoint of the drone if and only if the drone is alligned with the opening of the window. This can done either by setting the forward velocity of the drone when the error magnitude is below a certain threshold. This is however very slow in practice reducing the movement speed of the drone significantly. This is overcome by setting the pitch according to the inverse of the Magnitude of the Error vector. hence the more alligned the image frame is to the window detection, the more alligned the drone is to the window opening and the error vector has least magnitude, thus maximum forward velocity.

Mathematically, this model can be represented as:

![Error vec Eqns](/images/SetPointEquations.png)

We use standard PID controller to adjust the next states of Roll,Pitch,Yaw,Thrust in accordance to the abovementioned relation.

*The Error vector measured in a simulation containing a newly detected window at 1st Iteration:*
![feature points and genetrated SLAM map at the end of the mapping process](/images/control_start.png)

Once we have the Next states for Roll, Pitch, Yaw, Thrust, we need an algorithm to adjust the 6 motors for controlling these states. This is achieved by using a modified Quadcopter Motor Mixing controller where 4 motors are controlled with alternating rotation directions and a mapping function that maps the PID output values accordingly to the ESC limits of PWM signal. The modification is in the other 2 motors apart from the 4 motors being used to control states are solely used to control the vertical thrust in comparison to standard procedure where thrust value is coupled with every motor signal.

*The control algorihm minimizing the error vector after controlling the states for Nth Iteration:*
![feature points and genetrated SLAM map at the end of the mapping process](/images/control_final.png)
