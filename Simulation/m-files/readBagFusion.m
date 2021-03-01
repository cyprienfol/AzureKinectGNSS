% Read Fixposition sensor Fusion Topic from the BAG file 
%   readBagFusion.m 
%
%   See also CELLFUN matlab built-in function, 
%            SELECT, READMESSAGES matlab ROS TOOLBOX
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function Fusion = readBagFusion(bag, topic)
% inputs (2)  : - A rosbag or bag is a file format for storing ROS message data.
%                 MATLAB provides functionality for reading existing rosbags.
%
%               - topic is the name of the GNSS ROS topic.
%                 topic must be specified as a string or char
%
% outputs (1): - Fusion is a struct that regroups all the information 
%                relative to the proprietary Fusion algorithm of Fixposition
%

%Extract the GNSS topic from the Rosbag 
BagFusion = select(bag, 'Topic', topic);
Fusion.rosmessage = readMessages(BagFusion,"DataFormat","struct");

%Read the bag file and sort data for trajectory
%Convert the time of rosbag to UNIX
Fusion.timestamp = cellfun(@(m) double(m.Header.Stamp.Sec), Fusion.rosmessage) + ...
                            (10^-9)*cellfun(@(m) double(m.Header.Stamp.Nsec), Fusion.rosmessage);  

%ECEF geodetic trajectory
Fusion.position(:,1) = cellfun(@(m) double(m.Pose.Pose.Position.X),Fusion.rosmessage);
Fusion.position(:,2) = cellfun(@(m) double(m.Pose.Pose.Position.Y),Fusion.rosmessage);
Fusion.position(:,3) = cellfun(@(m) double(m.Pose.Pose.Position.Z),Fusion.rosmessage);

Fusion.quaternion(:,1) = cellfun(@(m) double(m.Pose.Pose.Orientation.X),Fusion.rosmessage);
Fusion.quaternion(:,2)= cellfun(@(m) double(m.Pose.Pose.Orientation.Y),Fusion.rosmessage);
Fusion.quaternion(:,3) = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),Fusion.rosmessage);
Fusion.quaternion(:,4) = cellfun(@(m) double(m.Pose.Pose.Orientation.W),Fusion.rosmessage);
% Fusion.orientation.Linear(:,1) = cellfun(@(m) double(m.Twist.Twist.Linear.X),Fusion.rosmessage);
% Fusion.orientation.Linear(:,2)= cellfun(@(m) double(m.Twist.Twist.Linear.Y),Fusion.rosmessage);
% Fusion.orientation.Linear(:,3) = cellfun(@(m) double(m.Twist.Twist.Linear.Z),Fusion.rosmessage);
% 
% Fusion.orientation.Angular(:,1) = cellfun(@(m) double(m.Twist.Twist.Angular.X),Fusion.rosmessage);
% Fusion.orientation.Angular(:,2)= cellfun(@(m) double(m.Twist.Twist.Angular.Y),Fusion.rosmessage);
% Fusion.orientation.Angular(:,3) = cellfun(@(m) double(m.Twist.Twist.Angular.Z),Fusion.rosmessage);
end