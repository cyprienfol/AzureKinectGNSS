# Simulation

## Overview

This repository contains the data and files used to fuse the vision-RTK IMU and GNSS receivers. The performance of the sensor fusion algorithm is then assessed thanks to the Fixposition solution fusing the IMU, GNSS receivers and the visual inputs from a monochrome camera. In addition, the simulation of a laser mounted on the platform is done using the navigationnal information from Fixposition.  

The 3 first folders are named after the 3 experiments conducted with the Kuka Robotic arms (ref. Ch. 7 Master thesis). They contains the raw data from the Platform (Azure Kinect + Vision-rtk)  either encrypted in a .bag file or  a .mat file formats. Also, the raw data of the Kuka robot are present in a .txt file or in a .mat file formats.

The m-files folder contains the matlab library developped during my master thesis and necessary to run the code in mainKalman.m performing the sensor fusion of the sensors mounted on the platform. 
    
## Screenshots
<img src="GNSS-IMU/Images/Laser_simulation.jpg" width="400">
<img src="GNSS-IMU/Images/Target_hit.jpg" width="400">
<img src="GNSS-IMU/Images/Fixposition_comparison.jpg" width="450">
<img src="GNSS-IMU/Images/Ellipse_confidence.jpg" width="400">


## Software requirement
Runnig has been tested on Ubuntu 18.04 with Matlab R2020b.

This repository, and the matlab ROS Toolbox are required to run the mainKalman scripts. 

## Running

If the software requirement are met, then, the mainSimulation.m could be opened in Matlab. 
The green button run can be pressed to launch the software. After a few seconds, 9 graphs should appear on the screen showing a visual comparison between our sensor fusion algorithm and the Fixposition solution. The five last graph correspond to the laser simulation. In addition, in the command window the statistical information from the sensors fusion results are displayed.

By default, the dataset from the Infinite Motion experiment is loaded but this could be changed very easily. By changing line 24,25 the parameters nameMotion and startTime accordingly to the folders information containing the raw data.

Note: The scale and font were addapted to fit the report layout. However, it is very easy to modified these parametes directly from the graph,(please ref. to the matlab documentation)

 
