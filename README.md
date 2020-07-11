# Zephyrus - Drone System for Window Detection and Autonomous Control.
This software repository contains the software frameworks and source code to the various algorithms that collectively make the system exhibit an ideal autonomous behaviour and window detection algorithm required for **Flipkarts GRID2.0 competition.**

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
