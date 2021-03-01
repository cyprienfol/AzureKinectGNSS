% Read IMU TOPIC from the BAG file 
%   readBagIMU.m 
%
%   See also TIMESERIES matlab built-in function, 
%            READMESSAGES, SELECT,  matlab ROS TOOLBOX
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function Imu = readBagIMU(bag, topic)
% inputs (2)  : - A rosbag or bag is a file format for storing ROS message data.
%                 MATLAB provides functionality for reading existing rosbags.
%
%               - topic is the name of the IMU ROS topic.
%                 topic must be specified as a string or char
%
% output (1)  : - IMU is a struct regrouping all the information related to
%                the IMU sensor. (ref. Chapter 4 Report)  

%read the selected rostopic
BagIMU = select(bag, 'Topic', topic);
Imu.rosmessage = readMessages(BagIMU,"DataFormat","struct");

%Extract accelerometer information as a timeseries
Imu.timeserie = timeseries(BagIMU, 'LinearAcceleration.X', 'LinearAcceleration.Y', 'LinearAcceleration.Z',...
                          'AngularVelocity.X', 'AngularVelocity.Y', 'AngularVelocity.Z');

%Parse information contained in the timeserie
Imu.specificforce = Imu.timeserie.Data(:,1:3);
Imu.angularrate = Imu.timeserie.Data(:,4:6);

%Add the timestamp to the trajectory
Imu.timestamp = Imu.timeserie.Time;

end