% Convert the XYZ local coordinate into ENU local coordinate 
%   convertLocalToEnu.m 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function enu = convertLocalToEnu(local)
% input (1)  : - local is a nx3 vector, each column represents a coordinate
%                from the XYZ local coordinate system.
%
% output (1) : - enu is a nx3 vector, each column represents a coordinate
%                from the East Nort UP coordinate system.
%

%Rotation around the z axis was define thanks to the Linear Dataset
Rz = @(Psi)[cos(Psi), sin(Psi), 0; -sin(Psi), cos(Psi), 0; 0, 0, 1];
enu = (Rz(6/7*pi)\(local'))';

end