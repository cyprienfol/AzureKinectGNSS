% Convert the ENU local coordinate into XYZ Cartesian coordinate 
%   convertEnuToCartesian.m 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%
function cartesian = convertEnuToCartesian(enu, refPoint)
% inputs (2) : - enu is a nx3 vector, each column represents a coordinate
%                from the East Nort UP coordinate system.
%
%              - refPoint is 1x3 vector representin the origin of the East North Up coordinate system
%
% output (1) : - cartesian is the nx3 vector, each column represents a coordinate
%                of the cartesian coordinate system
%


%Create the Roation matrix
RotationMatrix = [-sind(refPoint.longitude), cosd(refPoint.longitude), 0;...
                 -sind(refPoint.latitude)*cosd(refPoint.longitude), -sind(refPoint.latitude)*sind(refPoint.longitude), cosd(refPoint.latitude);...
                 cosd(refPoint.latitude)*cosd(refPoint.longitude), cosd(refPoint.latitude)*sind(refPoint.longitude), sind(refPoint.latitude)];

%Bring the ENU to the Cartesian coordinate frame. 
cartesian = (RotationMatrix\(enu'))' + (refPoint.xyz);

end