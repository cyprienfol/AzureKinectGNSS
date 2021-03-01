% Synchronize the data of the experiments (KUKA; VISIONRTK)  
%   synchroniseData.m 
%
%   See also MEDIAN, SUM matlab built-in function, 
%            SYNCHRONISEGNSS, SYNCHRONISEIMU   
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.

function [Kuka_KF, VisionRTK_KF] = synchroniseData(Kuka, VisionRTK, timeshift)
% inputs (4)  : - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%               
%               - AzureKinect is a struct regrouping all the information related to
%                the Azure Kinect measurement. (ref. Ch.4 of report)
%
%               - VisionRTK is a struct regrouping all the information related to
%                the vision-RTK measurement. (ref. Ch.4 of report)
%               
%               - timeshift is the timedifference between kuka measurement
%                 and the platform measurement. timeshift must be specified
%                 as a double 
%
% outputs (3) : - Kuka_KF is a struct regrouping all the information related to
%                the Kuka robotic arm measurement after preprocessing. (ref. Ch.4 of report)
%               
%               - AzureKinect_KF is a struct regrouping all the information related to
%                the Azure Kinect measurement after preprocessing. (ref. Ch.4 of report)
%
%               - VisionRTK_KF is a struct regrouping all the information related to
%                the vision-RTK measurement after preprocessing. (ref. Ch.4 of report)

%Relative time for kuka
Kuka_KF.trajectory.position = Kuka.trajectory.position - Kuka.trajectory.position(1,:);
Kuka_KF.trajectory.orientation = Kuka.trajectory.orientation - Kuka.trajectory.orientation(1,:);
Kuka_KF.timestamp = Kuka.timestamp - Kuka.timestamp(1,1);

%Synchronize GNSS with Kuka time  
timeGnss1_ = VisionRTK.Gnss1.timestamp + timeshift - Kuka.timestamp(1,1);
timeGnss2_ = VisionRTK.Gnss2.timestamp + timeshift - Kuka.timestamp(1,1);

%Remove data recorded before ground truth
startKukaGnss1 = sum(timeGnss1_<0) + 1;
startKukaGnss2 = sum(timeGnss2_<0) + 1;

%Synchronize Gnss receivers
[startGnss1, endGnss1, startGnss2, endGnss2] = synchroniseGnss(timeGnss1_(startKukaGnss1:end), timeGnss2_(startKukaGnss2:end));

startSyncGnss1 = startKukaGnss1 + startGnss1 - 1;
startSyncGnss2 = startKukaGnss2 + startGnss2 - 1;
endSyncGnss1 = startKukaGnss1 + endGnss1 - 1;
endSyncGnss2 = startKukaGnss2 + endGnss2 - 1;

%Synchronize Imu with Kuka time
timeImu_ = VisionRTK.Imu.timestamp + timeshift - Kuka.timestamp(1,1);

%Remove data recorded before ground truth
startKukaImu = sum(timeImu_<0) + 1;

%Synchronize Imu with Gnss receivers
[startImu, startGnss] = synchroniseImu(timeImu_(startKukaImu:end), timeGnss1_(startSyncGnss1:endGnss1));

startGnssImu = startKukaImu + startImu - 1;
startImuGnss1 = startSyncGnss1 + startGnss - 1;
startImuGnss2 = startSyncGnss2 + startGnss - 1;


%Get Data that correspond to the new timestamp
VisionRTK_KF.Imu.timestamp = timeImu_(startGnssImu:end);
VisionRTK_KF.Imu.specificforce = VisionRTK.Imu.specificforce(startGnssImu :end,:);
VisionRTK_KF.Imu.angularrate = VisionRTK.Imu.angularrate(startGnssImu:end,:);

VisionRTK_KF.Gnss1.timestamp = timeGnss1_(startImuGnss1:endSyncGnss1);
VisionRTK_KF.Gnss1.latitude = VisionRTK.Gnss1.latitude(startImuGnss1:endSyncGnss1);
VisionRTK_KF.Gnss1.longitude = VisionRTK.Gnss1.longitude(startImuGnss1:endSyncGnss1);
VisionRTK_KF.Gnss1.altitude = VisionRTK.Gnss1.altitude(startImuGnss1:endSyncGnss1);

VisionRTK_KF.Gnss2.timestamp = timeGnss2_(startImuGnss2:endSyncGnss2);
VisionRTK_KF.Gnss2.latitude = VisionRTK.Gnss2.latitude(startImuGnss2:endSyncGnss2);
VisionRTK_KF.Gnss2.longitude = VisionRTK.Gnss2.longitude(startImuGnss2:endSyncGnss2);
VisionRTK_KF.Gnss2.altitude = VisionRTK.Gnss2.altitude(startImuGnss2:endSyncGnss2);

end