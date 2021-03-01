% Evaluate the transformation applid to GNSS measurements
%   evaluateGNSS.m 
%
%   See also VISUALISEGNSS, COMPAREGNSS 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function evaluateGNSS(Kuka, VisionRTK)
% inputs (2) : - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - VisionRTK is a struct regrouping all the information related to
%                the vision-RTK measurement. (ref. Ch.4 of report)      

visualiseGNSS(VisionRTK);
compareGNSS(Kuka, VisionRTK);

end