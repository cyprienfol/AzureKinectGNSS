% Synchronize the IMU and GNSS timeseries
%   synchroniseImu.m 
%
%   See also SYNCHRONISEMEASUREMENT   
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.

function [startIndImu, startIndGnss] = synchroniseImu(timeImu, timeGnss)
% inputs (2)  : - timeImu is the timesteps vectors of the timeseries
%                 from the GNSS receivers
%
%               - timeGNSS is the timesteps vectors of the timeseries
%                 from the GNSS receivers
%
% outputs (4) : - startIndIMU is the index of the IMU sensor first measurement synchronized with 
%                 the both GNSS receiver measurement.
%
%               - startIndGNSS is the index of the GNSS receivers first measurement synchronized with 
%                 the the IMU sensor measurement.

%Initialise the Index for the timeseries
[startIndImu, startIndGnss] = synchroniseMeasurement(timeImu, timeGnss);

end