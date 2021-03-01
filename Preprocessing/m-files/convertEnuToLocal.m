% Convert the ENU local coordinate into XYZ local coordinate 
%   convertEnuToLocal.m 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function local = convertEnuToLocal(enu)
% input (1)  : - enu is a nx3 vector, each column represents a coordinate
%                from the East Nort UP coordinate system.
%
% output (1) : - local is a nx3 vector, each column represents a coordinate
%                from the XYZ local coordinate system.


%Rotation around the z axis was define thanks to the Linear Dataset
Rz = @(Psi)[cos(Psi), sin(Psi), 0; -sin(Psi), cos(Psi), 0; 0, 0, 1];
local = Rz(6/7*pi)*(enu');

end