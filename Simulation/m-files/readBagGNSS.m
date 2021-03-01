% Read GNSS Topic from the BAG file 
%   readBagGNSS.m 
%
%   See also CELLFUN matlab built-in function, 
%            SELECT, READMESSAGES matlab ROS TOOLBOX
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function Gnss = readBagGNSS(bag, topic)
% inputs (2)  : - A rosbag or bag is a file format for storing ROS message data.
%                 MATLAB provides functionality for reading existing rosbags.
%
%               - topic is the name of the GNSS ROS topic.
%                 topic must be specified as a string or char
%
% output (1)  : - GNSS is a struct regrouping all the information related to
%                the GNSS receiver from the vision-RTK. (ref. Chapter 4 Report)     


%Extract the GNSS topic from the Rosbag 
BagGNSS = select(bag, 'Topic', topic);
Gnss.rosmessage = readMessages(BagGNSS,"DataFormat","struct");

%Read the bag file and sort data for trajectory
%Convert the time of rosbag to UNIX
Gnss.timestamp = cellfun(@(m) double(m.Header.Stamp.Sec), Gnss.rosmessage) + ...
                            (10^-9)*cellfun(@(m) double(m.Header.Stamp.Nsec), Gnss.rosmessage);  

%ECEF geodetic trajectory
Gnss.latitude = cellfun(@(m) double(m.Latitude),Gnss.rosmessage);
Gnss.longitude = cellfun(@(m) double(m.Longitude),Gnss.rosmessage);
Gnss.altitude = cellfun(@(m) double(m.Altitude),Gnss.rosmessage);

end