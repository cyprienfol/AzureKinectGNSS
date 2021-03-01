# Kalman

## Overview

This repository contains the data and files used to fuse the Azure Kinect IMU and the Vision-RTK GNSS receivers. The sensor fusion is made with a Kalman Filter inspired from the architecture of Loosely Coupled Integration. The state vector is composed the (position, velocity, acceleration, orientation, angular rate, accelerometer bias and gyroscope bias)^T and the measurement vector is made of (Baseline center Global position, Baseline yaw orientation, IMU specific force, IMU angular rate)^T.

The 3 first folders are named after the 3 experiments conducted with the Kuka Robotic arms (ref. Ch. 7 Master thesis). They contains the raw data from the Platform (Azure Kinect + Vision-rtk)  either encrypted in a .bag file or  a .mat file formats. Also, the raw data of the Kuka robot are present in a .txt file or in a .mat file formats.

The m-files folder contains the matlab library developped during my master thesis and necessary to run the code in mainKalman.m performing the sensor fusion of the sensors mounted on the platform. 
    
## Screenshots
<img src="GNSS-IMU/Images/Position_fusion.jpg" width="450">
<img src="GNSS-IMU/Images/Residual_position.jpg" width="450">
<img src="GNSS-IMU/Images/Orientation_fusion.jpg" width="450">
<img src="GNSS-IMU/Images/Residual_orientation.jpg" width="450">

## Software requirement
Runnig has been tested on Ubuntu 18.04 with Matlab R2020b.

This repository, and the matlab ROS Toolbox are required to run the mainKalman scripts. 

## Running

If the software requirement are met, then, the mainKalman.m could be opened in Matlab. 
The green button run can be pressed to launch the software. After a few seconds, 5 graphs should appear on the screen showing the performance of the sensor fusion and comparing it to the Ground Truth (Kuka). In addition, in the command window the statistical information from the sensors fusion results are displayed.

By default, the dataset from the Infinite Motion experiment is loaded but this could be changed very easily. By changing line 21,22 the parameters nameMotion and startTime accordingly to the folders information containing the raw data.

Note: The scale and font were addapted to fit the report layout. However, it is very easy to modified these parametes directly from the graph,(please ref. to the matlab documentation)

 

 




