% Integrate the IMU to get the TRAJECTORY (position + orientaiton)of the platform  
%   integrateIMU.m 
%
%   See also CUMSUM matlab built-in function, 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function trajectory = integrateIMU(Imu, Time)
% input (2)  : - Imu is a struct that regroup all the information from
%                IMU sensor
%
%              - Time is the timesteps vectors of the timeseries
%                from the IMU measurements
%
% outputs (1): - trajectory is a struct that regroups all the navigationnal
%                information of the IMU after integration (position and
%                velocity)
%

%Parameter definition
gravity = [0,0,9.81];
dt = diff(Time);

%Integration of specific force to get the motion
trajectory.acceleration = Imu.specificforce + gravity;
trajectory.velocity = cumsum(trajectory.acceleration(1:end-1,:).*dt);
trajectory.position = cumsum(trajectory.velocity(1:end-1,:).*dt(2:end) + (1/2)*trajectory.acceleration(2:end-1,:).*(dt(2:end).^2));

%Integration of the angular rate to get the rotation information 
trajectory.angularrate = Imu.angularrate;
trajectory.attitude = [0,0,0;cumsum(trajectory.angularrate(1:end-1,:).*dt)];


end