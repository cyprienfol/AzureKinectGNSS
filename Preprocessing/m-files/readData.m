% Read Data associated to STARTTIME in the FOLDERDATA.
%   readData.m 
%
%   See also CD, DIR, matlab built-in function, 
%            READLOGKUKA, READLOGPLATFORM
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%
function [Kuka, AzureKinect, VisionRTK]= readData(FolderData, startTime)
% inputs (2)  : - FolderData is the name of the folder where the raw data
%                 are stored, FolderData must be specified as a character
%                 or a string 
%
%               - startTime is the time when the measurements took place,
%                 startTime must be specified as a scalar string
%
% outputs (3): - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - AzureKinect is a struct regrouping all the information related to
%                the Azure Kinect measurement. (ref. Ch.4 of report)
%
%              - VisionRTK is a struct regrouping all the information related to
%                the vision-RTK measurement. (ref. Ch.4 of report)
%


%Enter the Data directory
cd(FolderData);

%Extract Path information 
infoLogKuka = dir("Kuka*");
infoLogPlatform = dir("Platform*");

%return to the main directory
cd ..

%% Load GroundTruth Data
KukaFolder = infoLogKuka.folder;
KukaFile = infoLogKuka.name;
Kuka = readLogKuka(KukaFolder, KukaFile, startTime);

%% Load Platform Data
PlatformFolder = infoLogPlatform.folder;
PlatformFile = infoLogPlatform.name;
[AzureKinect, VisionRTK] = readLogPlatform(PlatformFolder, PlatformFile, startTime);     

end