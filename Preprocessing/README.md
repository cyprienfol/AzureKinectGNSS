# Preprocessing

## Overview

This repository contains the data and files used to preprocess the data of all devices during the experiments: Azure Kinect, Vision-RTK and Kuka KR6 900. The data from all sensors are synchronised and the appropriate trasnformation are applied in order to prepare them for the sensor fusion. 
 
The 3 first folders are named after the 3 experiments conducted with the Kuka Robotic arms (ref. Ch. 7 Master thesis). They contains the raw data from the Platform (Azure Kinect + Vision-rtk)  either express in a .bag file or  a .mat file formats. Also, the raw data of the Kuka robot are present in a .txt file or in a .mat file formats.

The m-files folder contains the matlab library developped during my master thesis and necessary to run the code written in mainPreprocessing.m.
    
## Screenshots
<img src="https://github.com/cyprienfol/AzureKinectGNSS/blob/main/Images/Timeshift.jpg" width="450">
<img src="https://github.com/cyprienfol/AzureKinectGNSS/blob/main/Images/GNSS_transformation.jpg" width="450">
<img src="https://github.com/cyprienfol/AzureKinectGNSS/blob/main/Images/IMU_transformation.jpg" width="450">
<img src="https://github.com/cyprienfol/AzureKinectGNSS/blob/main/IMU_comparison.jpg" width="450">


## Software requirement
Runnig has been tested on Ubuntu 18.04 with Matlab R2020b.

This repository, and the matlab ROS Toolbox are required to run the mainPreprocessing script. 

## Running

If the software requirement are met, then, the mainPreprocessing.m could be opened in Matlab. 
The green button run can be pressed to launch the software. After a few seconds, 18 graphs should appear on the screen showing the results after preprocossing and the comparison between the IMU of Vision-RTK and Azure Kinect. In addition, in the command window the offset from the Kuka robot axis and the statistical information from the IMUs are displayed

By default, the dataset from the Infinite Motion experiment is loaded but this could be changed very easily. By changing line 27,28 the parameters nameMotion and startTime accordingly to the folders containing the raw data.

Note: The scale and font were addapted to fit the report layout. However, it is very easy to modified these parameters directly from the graph options (please ref. to the matlab documentation).
