% Preprocess the data from .bag file (Sensors) and the .txt file
% (Reference).
%   mainPreprocessing.m 
%
%   See also READDATA, ESTIMATEPARAMETER, SYNCHRONIZEKUKAIMU, ...
%            INTEGRATEIMU, MECHANISEIMU, CONVERTIMUAZURETOLOCAL, ...
%            CONVERTGEODETICTOCARTESIAN, CONVERTCARTESIANTOLOCAL,CONVERTENUTOLOCAL...
%            COMPAREIMU, EVALUATEIMU, COMPAREGNSS, EVALUATEGNSS, ...
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.


%% Worspace Preparation
%clean workspace
clear 
close all
clc

%Add the matlab library of Cyprien FOL 
addpath('m-files')


%% Parameters
%Name of the folder where to look for the data
nameMotion = 'Infinite';
startTime = '2020_11_05_1625';
dirData = [nameMotion, 'Motion', '_', startTime];

%% Load data 
%Read sensors data from the'm Folder in dirData
[Kuka, AzureKinect, VisionRTK]= readData(dirData, startTime);

%Visualize Ground Truth
visualiseKuka(Kuka);

%Estimate parameter of the dataset determine during postprocessinf
Parameter  = estimateParameter(VisionRTK, nameMotion, startTime);

%% PARTI Preprocessing of raw data
%Synchronisation of Imu data with groundtruth
syncImuAzure = synchroniseKukaImu(Kuka.timestamp, AzureKinect.Imu, Parameter.timeshift);
syncImuVision = synchroniseKukaImu(Kuka.timestamp, VisionRTK.Imu, Parameter.timeshift);

syncImuAzure.integration = integrateIMU(syncImuAzure, syncImuAzure.timestamp);
syncImuVision.integration = integrateIMU(syncImuVision, syncImuVision.timestamp);

%Convert IMU Data into the same localframe
ImuAzure = convertBodyToLocal(syncImuAzure); 
ImuVision = convertBodyToLocal(syncImuVision);

%IMU mechanisation
ImuAzure.integration = mechaniseIMU(ImuAzure, Parameter);
ImuVision.integration = mechaniseIMU(ImuVision, Parameter);

%Plot oriented measurement IMU
evaluateIMU(syncImuAzure,ImuAzure, 'azure');
evaluateIMU(syncImuAzure,ImuAzure, 'vision');


%Conversion from geodetic ECEF to cartesian ECEF Coordinate system
VisionRTK.Gnss1.trajectory.cartesian = convertGeodeticToCartesian(VisionRTK.Gnss1);
VisionRTK.Gnss2.trajectory.cartesian = convertGeodeticToCartesian(VisionRTK.Gnss2);

%Conversion from cartesian ECEF to ENU Coordinate system
VisionRTK.Gnss1.trajectory.ENU = convertCartesianToEnu(VisionRTK.Gnss1.trajectory.cartesian, Parameter.center);
VisionRTK.Gnss2.trajectory.ENU = convertCartesianToEnu(VisionRTK.Gnss2.trajectory.cartesian, Parameter.center);

%Conversion from ENU to Local Coordinate system
VisionRTK.Gnss1.trajectory.local = convertEnuToLocal(VisionRTK.Gnss1.trajectory.ENU)';
VisionRTK.Gnss2.trajectory.local = convertEnuToLocal(VisionRTK.Gnss2.trajectory.ENU)';

%Evaluation of the GNSS acccuracy
evaluateGNSS(Kuka, VisionRTK);

%% PART II: Comparison of the IMUs
%Evaluation of the IMU accuracy 
%Comparison and evaluation of the IMU on the platform
compareIMU(Kuka, ImuAzure.integration, ImuAzure.timestamp, ImuVision.integration, ImuVision.timestamp);       
 


