% Compare visually the AZUREIMU and VISIONIMU with plots, and perform basic statistical analysis
%   compareIMU.m 
%
%   See also COMPAREGYRO, COMPAREACCEL 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function compareIMU(Kuka, AzureImu, AzureTime, VisionImu, VisionTime)
% inputs (5) : - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - AzureImu is a struct that regroups all the measurements
%                from the Azure Kinect IMU
%
%              - AzureTime is 1xn double vector that represents each timesteps 
%                the Azure Kinect IMU was taking measurements
%
%              - VisionImu is a struct that regroups all the measurements
%                from the Vision-RTK IMU
%
%              - VisionTime is 1xn double vector that represents each timesteps 
%                the Vision-RTK IMU was taking measurements
%

%% PART I: Compare Gyroscopes on the platform with Kuka Robot (Ground truth Data) 
compareAccel(Kuka, AzureImu.position, AzureTime(3:end), VisionImu.position, VisionTime(3:end));

%% PART II: Compare Accelerometer on the platform with the Kuka 
compareGyro(Kuka, AzureImu.attitude, AzureTime, VisionImu.attitude, VisionTime);


end