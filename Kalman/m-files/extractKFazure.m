% Extract specific force, angularrate and biases for the IMU of AZUREKINECT
%   exctratKFazure.m 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [f, omega, b_acc, b_gyro] = extractKFazure(AzureKinect)
% input (1)  : - AzureKinect is a struct regrouping all the information related to
%                the Azure Kinect measurement. (ref. Ch.4 of report)
%
% outputs (4): - f is nx3 double vector that represent the specific force
%                 from the Azure Kinect IMU
%
%              - omega is nx3 double vector that represent the angular
%                rate from the Azure Kinect IMU
%
%              - b_acc is nx3 double vector that represent the biases
%                of the accelerometer from the Azure Kinect IMU
%
%              - b_gyro is nx3 double vector that represent the biases
%                of the gyroscope from the Azure Kinect IMU
%


%Extract data for IMU
f = AzureKinect.Imu.local.specificforce; 
omega = AzureKinect.Imu.local.angularrate;

%Initialise the parameters KF3
b_acc = 0.012; %12[mm/s^2]
b_gyro = 0.001; %~100[deg/h]

end