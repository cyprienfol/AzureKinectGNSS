% Sensor fusion using different variation of Kalman Filter
%   mainKalman.m 
%
%   See also READDATA, ESTIMATEPARAMETER, SYNCHRONIZEKUKAIMU, ...
%            INTEGRATEIMU, MECHANISEIMU, CONVERTIMUAZURETOLOCAL, ...
%            CONVERTGEODETICTOCARTESIAN, CONVERTCARTESIANTOLOCAL,CONVERTENUTOLOCAL...
%            EXTRACTKFDATA, FUSEIMUGNSS, 
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
startTime = '2020_11_05_1625';
dirData = [nameMotion,'Motion','_',startTime];

%Add the matlab library of Cyprien FOL 
addpath('m-files')

%define which version of the kalman filter to use
flagkf = 0;

%% Load data 
%Read sensors data from the Folder in dirData
[Kuka, AzureKinect, VisionRTK]= readData(dirData, startTime);

%Estimate parameter of the dataset determine during postprocessinf
Parameter  = estimateParameter(VisionRTK, nameMotion, startTime);

%Synchronization of the sensors
[Kuka_KF, AzureKinect_KF, VisionRTK_KF] = synchroniseData(Kuka, AzureKinect, VisionRTK, Parameter.timeshift);

%Prepare variable for compacity of the algorithm
%Cartesian ECEF Coordinaten(x,y,z)
VisionRTK_KF.Gnss1.cartesian = convertGeodeticToCartesian(VisionRTK_KF.Gnss1);
VisionRTK_KF.Gnss2.cartesian = convertGeodeticToCartesian(VisionRTK_KF.Gnss2);

%Align data of Imu to ENU
AzureKinect_KF.Imu.local = convertBodyToLocal(AzureKinect_KF.Imu);

%Extract for the Kalman filter
[f, omega, b_acc, b_gyro, x_local1, x_local2] = extractKFdata(AzureKinect_KF, VisionRTK_KF, Parameter);

%Fuse data with a Linear Kalman Filter
Fusion = fuseIMUGNSS(f, omega, x_local1, x_local2, b_acc, b_gyro, AzureKinect_KF.Imu.timestamp, VisionRTK_KF.Gnss1.timestamp, Parameter, flagkf);

%Analyse Data and create Plots
visualiseFusion(Fusion, Kuka_KF, Parameter);


