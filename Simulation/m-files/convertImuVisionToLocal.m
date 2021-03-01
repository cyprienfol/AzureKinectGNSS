% Convert IMU data from the body frame to local frame  
%   convertImuVisionToLocal.m 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function local = convertImuVisionToLocal(Imu)
% input (1) : - Imu is a struct that regroup all the information from
%               IMU sensor
%
% output (1): - local is a struct containing the raw IMU measurement
%               transformed into local reference frame 
%

Rx = @(Phi)[ 1 0 0;...
                0 cos(Phi), sin(Phi);...
                0, -sin(Phi), cos(Phi)];
            
Ry = @(Theta)[cos(Theta), 0, -sin(Theta);...
                 0, 1, 0; ...
                 sin(Theta), 0, cos(Theta)];

Rz = @(Psi)[cos(Psi), sin(Psi), 0;...
               -sin(Psi), cos(Psi), 0;...
               0, 0, 1];
%Align axis
alpha = 0;
beta = pi;
gamma = pi/2;
local.specificforce = (Rz(3*pi/2) * Imu.specificforce')';
local.angularrate = (Rx(alpha)*Ry(beta)*Rz(gamma) * Imu.angularrate')';

%get the synchronised timestamp
local.timestamp = Imu.timestamp;

end