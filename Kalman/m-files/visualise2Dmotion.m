% Visualise with plots the position and orientation of the platform and
% Ground truth
%   visualise2Dmotion.m 
%
%   See also TIMESERIES, SYNCHRONIZE, QUIVER3, matlab built-in function
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function visualise2Dmotion(Kuka, Fuse)
% inputs (2) : - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - Fusion is a struct regrouping all the information related to
%                the sensor Fusion. (ref. Ch.4 of report)
%  

%Plots the 3 components of position
figure;
subplot(2,3,1)
plot(Kuka.timestamp, Kuka.trajectory.position(:,1), '.');
title('x_{platform}');
subplot(2,3,4)
plot(Fuse.timestamp, Fuse.position(1,:), '.');

subplot(2,3,2)
plot(Kuka.timestamp, Kuka.trajectory.position(:,2), '.');
title('y_{platform}');
subplot(2,3,5)
plot(Fuse.timestamp, Fuse.position(2,:), '.');

subplot(2,3,3)
plot(Kuka.timestamp, Kuka.trajectory.position(:,3), '.');
title('z_{platform}');
subplot(2,3,6)
plot(Fuse.timestamp, Fuse.position(3,:), '.');

sgtitle('Position')

%Plot the 3 component of orientation 
figure;
subplot(2,3,1)
plot(Kuka.timestamp,Kuka.trajectory.orientation(:,1), '.');
title('\Sigma_{x}');
subplot(2,3,4)
plot(Fuse.timestamp, Fuse.attitude(1,:), '.');

subplot(2,3,2)
plot(Kuka.timestamp,Kuka.trajectory.orientation(:,2), '.');
title('\Sigma_{y}');
subplot(2,3,5)
plot(Fuse.timestamp, Fuse.attitude(2,:), '.');

subplot(2,3,3)
plot(Kuka.timestamp,Kuka.trajectory.orientation(:,3), '.');
title('\Sigma_{z}');
subplot(2,3,6)
plot(Fuse.timestamp, Fuse.attitude(3,:), '.');

sgtitle('Attitude')

end