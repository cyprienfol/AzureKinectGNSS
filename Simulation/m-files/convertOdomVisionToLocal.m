% Convert Odometry data from the body frame to local frame  
%   convertOdomVisionToLocal.m 
%
%   See also QUAT2EUL matlab built-in function, 
%            CONVERTENUTOLOCAL
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function local = convertOdomVisionToLocal(Odometry)
% input (1) : - Odom is a struct that regroup all the information from
%               the visual odometry of vision-RTK 
%
% output (1): - local is a struct containing the raw IMU measurement
%               transformed into local reference frame 
%
%Declare the rotation matrix 
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
local.position = convertEnuToLocal(Odometry.position - Odometry.position(1,:));%convertEnuToLocal(Odometry.position-mean(Odometry.position(1:50,:)));


orientation = quat2eul(Odometry.quaternion);
offset_gyro = orientation(1,:);
local.attitude  = (orientation - offset_gyro)';

end