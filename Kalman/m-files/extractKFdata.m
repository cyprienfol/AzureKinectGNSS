% Extract the data from the IMU of AZUREKINECT and GNSS receivers of VISIONRTK 
% for the inputs of the Kalman filter
%   exctratKFdata.m 
%
%   See also EXTRACTKFAZURE, EXTRACTKFVISION  
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [f, omega, b_acc, b_gyro, x_local1, x_local2] = extractKFdata(AzureKinect, VisionRTK, Parameter)
% inputs (3)  : - AzureKinect is a struct regrouping all the information related to
%                the Azure Kinect measurement. (ref. Ch.4 of report)
%
%               - VisionRTK is a struct regrouping all the information related to
%                 the vision-RTK measurement. (ref. Ch.4 of report)
%
%               - Parameter contains the information influencing
%                the sensor fusion based on the experiment performed
%
% outputs (6) : - f is nx3 double vector that represent the specific force
%                 from the Azure Kinect IMU
%
%               - omega is nx3 double vector that represent the angular
%                 rate from the Azure Kinect IMU
%
%               - b_acc is nx3 double vector that represent the biases
%                 of the accelerometer from the Azure Kinect IMU
%
%               - b_gyro is nx3 double vector that represent the biases
%                 of the gyroscope from the Azure Kinect IMU
%
%               - x_local1 is nx3 double vector that represent the
%                 position of the left GNSS receiver in the local
%                 coordinate system
%
%               - x_local2 is nx3 double vector that represent the
%                 position of the right GNSS receiver in the local
%                 coordinate system
%
%

%Get starting Time of the experiment
[f, omega, b_acc, b_gyro] = extractKFazure(AzureKinect);
[x_local1, x_local2] = extractKFvision(VisionRTK, Parameter);

end