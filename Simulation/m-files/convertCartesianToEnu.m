% Convert the CARTESIAN ecef coordinate into ENU local frame using REFPOINT 
%   convertCartesianToEnu.m 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function enu = convertCartesianToEnu(cartesian, refPoint)
% inputs (2)  : - cartesian is the nx3 vector, each column represents a coordinate
%                of the cartesian coordinate system
%
%              - refPoint is 1x3 vector representin the origin of the East North Up coordinate system
%
% output  (1): - enu is a nx3 vector, each column represents a coordinate
%                from the East Nort UP coordinate system.
%


%Create the Roation matrix
RotationMatrix = [-sind(refPoint.longitude), cosd(refPoint.longitude), 0;...
                 -sind(refPoint.latitude)*cosd(refPoint.longitude), -sind(refPoint.latitude)*sind(refPoint.longitude), cosd(refPoint.latitude);...
                 cosd(refPoint.latitude)*cosd(refPoint.longitude), cosd(refPoint.latitude)*sind(refPoint.longitude), sind(refPoint.latitude)];
             
%Bring the cartesian coordinate to the ENU reference frame. 
enu = (RotationMatrix * (cartesian-refPoint.xyz)')'; 

end