% Synchronise the IMU data with the Ground truth (KUKA)
%   synchroniseKukaImu.m 
%
%   See also SYNCHRONISEMEASUREMENT
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.

function syncImu = synchroniseKukaImu(kukaTime, imu, timeshift)
% input (3)  : - kukaTime is the timesteps vectors of the timeseries
%                from the Kuka robot
%
%              - imu is a struct that regroup all the information from
%                IMU sensor
%
%              - timeshift is a double that represents the timeshift 
%                between the platform sensor and the Kuka robot
%
% output (1): - syncImu is a struct that regroups the the IMU measurement 
%                after synchronisation 
%

timeImu = imu.timestamp + timeshift;
%Count the number of data recorded before kuka. 
[startIndKuka,startIndImu] = synchroniseMeasurement(kukaTime, timeImu);

%Create the structure of the synchronizeImu
syncImu.timestamp = timeImu(startIndImu:end);
syncImu.specificforce = imu.specificforce(startIndImu:end, :);
syncImu.angularrate = imu.angularrate(startIndImu:end, :);



end