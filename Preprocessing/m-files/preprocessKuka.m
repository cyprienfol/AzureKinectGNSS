% Preprocess Kuka RAWDATA, convert the time from Modified Julian formate to UNIX
% and convert RAWDATA into S.I units 
%   preprocessKuka.m 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%
function [timestamp, trajectory] = preprocessKuka(rawData)
% input (1)  : - rawData is the parse measurment from the Kuka .txt file 
%
% outputs (2): - timestamp is the time associated to each measurement taken
%                by the Kuka robot. It is expressed in UNIX time 
%
%              - trajectory are position and orientation information for
%                the Kuka robot (Ground Truth)
     
%% Parse the raw data
MJD = rawData(:,1);
sec = rawData(:,2);
X = rawData(:,3);
Y = rawData(:,4);
Z = rawData(:,5);
A = rawData(:,6);
B = rawData(:,7);
C = rawData(:,8);

%Time conversion
JD = MJD + 2400000.5;
UNIX = (JD - 2440587.5) * 86400; 
timestamp = UNIX + sec;

%Position
trajectory.position(:,1) = X;
trajectory.position(:,2) = Y;
trajectory.position(:,3) = Z;

%Conversion to S.I unit (mm->m)
trajectory.position = (1/1000)*trajectory.position;

%Orientation
trajectory.orientation(:,3) = A; %Rotation around the z-axis
trajectory.orientation(:,2) = B; %rotation around the y-axis
trajectory.orientation(:,1) = C; %Rotaiton aound the x-axis

%Conversion to S.I unit (°->rad)
trajectory.orientation = (pi/180)*trajectory.orientation;

end