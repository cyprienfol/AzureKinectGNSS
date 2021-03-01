% Convert the ECEF geodetic coordinate to ECEF cartesian coordinate 
%   convertGeodeticToCartesian.m 
%
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.

function cartesian = convertGeodeticToCartesian(ecef)
% input (1)  : - ecef is a struct representing the coordinate
%                of the ECEF geodetic coordinate system
%
% output (1): - cartesian is the nx3 vector, each column represents a coordinate
%                of the ECEF cartesian coordinate system
%
       

%Define parameters
lambda = ecef.longitude;%[°]
Phi = ecef.latitude;%[°]
h = ecef.altitude;%[°]

%Initialize constante
e_sqr = 6.69437999e-3;
a = 6378137;
N = a./sqrt(1 - e_sqr*(sind(Phi).^2));

%Calculate the cartesian coordinates
x_GPS = (N + h).*cosd(Phi).*cosd(lambda);
y_GPS = (N + h).*cosd(Phi).*sind(lambda);
z_GPS = ((1 + e_sqr)*N + h).*sind(Phi);

cartesian = [x_GPS, y_GPS, z_GPS];

end