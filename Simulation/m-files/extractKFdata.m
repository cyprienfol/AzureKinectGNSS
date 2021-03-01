% Extract the data from the IMU and GNSS receivers of VISIONRTK 
% for the inputs of the Kalman filter
%   exctratKFdata.m 
%
%   See also CONVERTCARTESIANTOENU, CONVERTENUTOLOCAL 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [f, omega, b_acc, b_gyro, x_local1, x_local2] = extractKFdata(VisionRTK, Parameter)
% inputs (2)  : - VisionRTK is a struct regrouping all the information related to
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
%Extract data for IMU
f = VisionRTK.Imu.local.specificforce; 
omega = VisionRTK.Imu.local.angularrate;

%Initialise the parameters KF3
b_acc = 0.012; %12[mm/s^2]
b_gyro = 0.001; %~100[deg/h]

%Extract data for GNSS
%Cartesian ECEF Coordinaten(x,y,z)
x_ecef1 = VisionRTK.Gnss1.cartesian;
x_ecef2 = VisionRTK.Gnss2.cartesian;

x_enu1 = convertCartesianToEnu(x_ecef1, Parameter.center);
x_enu2 = convertCartesianToEnu(x_ecef2, Parameter.center);

x_local1 = convertEnuToLocal(x_enu1)';
x_local2 = convertEnuToLocal(x_enu2)';

end