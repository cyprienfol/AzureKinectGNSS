% En tête
% Cyprien Fol, Zürich
% Master Thesis
% Projected Augmented Reality using Sensor Fusion
% estimationRefPoint.m

function refPoint = estimateRefPoint(gnss,restPeriod)
% input (1)  : - refPoint is the point used as the origin for the transformation
%                from cartesian to ENU reference frame. refPoint is a
%                struct
%
% outputs (2):  - gnss is a struct regrouping all the information related to
%                the GNSS receiver from the vision-RTK. (ref. Chapter 4 Report)     
%
%              - restPeriod correspond to a time duration where the platform was
%                steady before the motion. resperiod must be specified as
%                an integer
%

%Calculate the initial position of the platform

%Platform is at rest the 50 first measurements
refPoint.latitude = median(gnss.latitude(1:restPeriod,1));
refPoint.longitude = median(gnss.longitude(1:restPeriod,1));
refPoint.altitude = median(gnss.altitude(1:restPeriod,1));
refPoint.xyz = convertGeodeticToCartesian(refPoint);

%Calculate the center of the baseline to get the ENU origin
% initialPosPlatform.latitude =  initialPosGnss1.latitude + 0.5*(initialPosGnss2.latitude - initialPosGnss1.latitude);
% initialPosPlatform.longitude =  initialPosGnss1.longitude + 0.5*(initialPosGnss2.longitude - initialPosGnss1.longitude);
% initialPosPlatform.altitude =  initialPosGnss1.altitude + 0.5*(initialPosGnss2.altitude - initialPosGnss1.altitude);


end