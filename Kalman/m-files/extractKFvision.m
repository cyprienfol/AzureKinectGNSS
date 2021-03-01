% Extract local coordinate of the 2 GNSS receivers on VISIONRTK
%   exctratKFvision.m 
%
%   See also CONVERTCARTESIANTOENU, CONVERTENUTOLOCAL  
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [x_local1, x_local2] = extractKFvision(VisionRTK, Parameter)
% inputs (2)  : - VisionRTK is a struct regrouping all the information related to
%                 the vision-RTK measurement. (ref. Ch.4 of report)
%
%               - Parameter contains the information influencing
%                the sensor fusion based on the experiment performed
%
% outputs (2) : - x_local1 is nx3 double vector that represent the
%                 position of the left GNSS receiver in the local
%                 coordinate system
%
%               - x_local2 is nx3 double vector that represent the
%                 position of the right GNSS receiver in the local
%                 coordinate system
%

%Extract data for GNSS in Cartesian ECEF Coordinaten(x,y,z)
x_ecef1 = VisionRTK.Gnss1.cartesian;
x_ecef2 = VisionRTK.Gnss2.cartesian;

%XYZ Cartesian to East North Up coordinate system 
x_enu1 = convertCartesianToEnu(x_ecef1, Parameter.center);
x_enu2 = convertCartesianToEnu(x_ecef2, Parameter.center);

%East North Up to XYZ local coordinate sysem
x_local1 = convertEnuToLocal(x_enu1)';
x_local2 = convertEnuToLocal(x_enu2)';

end