% Read Data associated to STARTTIME in the FOLDERDATA.
%   readVisionRTK.m 
%
%   See also CD, DIR, matlab built-in function, 
%            READLOGKUKA, READLOGVISIONRTK
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [Kuka, VisionRTK]= readVisionRTK(FolderData, startTime)
% inputs (2)  : - FolderData is the name of the folder where the raw data
%                 are stored, FolderData must be specified as a character
%                 or a string 
%
%               - startTime is the time when the measurements took place,
%                 startTime must be specified as a scalar string
%
% outputs (2): - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - VisionRTK is a struct regrouping all the information related to
%                the vision-RTK measurement. (ref. Ch.4 of report)
%

%Enter the Data directory
cd(FolderData);

%Extract Path information 
infoLogKuka = dir("Kuka*");
infoLogVisionRTK = dir("Platform*");

%return to the main directory
cd ..

%% Load GroundTruth Data
KukaFolder = infoLogKuka.folder;
KukaFile = infoLogKuka.name;
Kuka = readLogKuka(KukaFolder, KukaFile, startTime);

%% Load Platform Data
PlatformFolder = infoLogVisionRTK.folder;
PlatformFile = infoLogVisionRTK.name;
VisionRTK = readLogVisionRTK(PlatformFolder, PlatformFile, startTime);     

end