% Sensor fusion of the vision-RTK IMU and GNSS receivers
% The navigationnal data are then used to simulate the projetion
% of a laser from the sensor toward a vertical plane
%   mainSimulation.m 
%
%   See also READVISIONRTK, ESTIMATEPARAMETER, SYNCHRONIZEDATA ...
%            MECHANISEIMU, CONVERTODOMVISIONTOLOCAL, CONVERTIMUVISIONTOLOCAL, ...
%            CONVERTGEODETICTOCARTESIAN, CONVERTCARTESIANTOLOCAL,CONVERTENUTOLOCAL...
%            EXTRACTKFDATA, SIMULATELASER, COMPAREFUSION 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.


%% Worspace Preparation
%clean workspace
clear 
close all
clc

%% Parameters
%Name of the folder where to look for the data
nameMotion = 'Infinite';
startTime = '2020_11_05_1605';
dirData = [nameMotion,'Motion','_',startTime];

%Add the matlab library of Cyprien FOL 
addpath('m-files')

%% Load data 
%Read sensors data from the Folder in dirData
[Kuka, VisionRTK]= readVisionRTK(dirData, startTime);

%Estimate parameter of the dataset determine during postprocessinf
Parameter  = estimateParameter(VisionRTK, nameMotion, startTime);

%% Fuse IMU and GNSS1 + GNSS2
%Synchronization of the sensors
[Kuka_KF, VisionRTK_KF] = synchroniseData(Kuka, VisionRTK, Parameter.timeshift);

%Prepare variable for compacity of the algorithm
%Cartesian ECEF Coordinaten(x,y,z)
VisionRTK_KF.Gnss1.cartesian = convertGeodeticToCartesian(VisionRTK_KF.Gnss1);
VisionRTK_KF.Gnss2.cartesian = convertGeodeticToCartesian(VisionRTK_KF.Gnss2);

%Align data of Imu to ENU
VisionRTK_KF.Imu.local = convertImuVisionToLocal(VisionRTK_KF.Imu);

%Extract for the Kalman filter
[f, omega, b_acc, b_gyro, x_local1, x_local2] = extractKFdata(VisionRTK_KF, Parameter);

%Fuse data with a Linear Kalman Filter
Fusion = fuseIMUGNSS(f, omega, x_local1, x_local2, b_acc, b_gyro, VisionRTK_KF.Imu.timestamp, VisionRTK_KF.Gnss1.timestamp, Parameter);

%% Convert Fixposition Data
Fixposition = convertOdomVisionToLocal(VisionRTK.Fusion);
Fixposition.timestamp = VisionRTK.Fusion.timestamp + Parameter.timeshift - Kuka.timestamp(1,1); 

%% Compare the two fuse solution
compareSensorFusion(Kuka_KF, Fusion, Fixposition);

%% Simulate the laser 
%Target parameter
Target.position = [3;0;0];
Target.normal = [-1;0;0];% + Target.position;

[Fixposition_hit, Fixposition_correction] = simulateLaser(Fixposition, Kuka_KF, Target);
% [Mechanisation_hit, Mechanisation_correction] = simulateLaser(Fuse, Kuka_KF, Target);
compareProjection(Fixposition_hit, Target);



