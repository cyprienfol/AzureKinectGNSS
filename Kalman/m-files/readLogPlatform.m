% Read Sensors bag. DATAFILE associated to STARTTIME in DATAFOLDER 
%   readLogPlatform.m 
%
%   See also FULLFILE, ISFILE, LOAD, SAVE matlab built-in function, 
%            ROSBAG matlab ROS TOOLBOX
%            READBAGIMU, READBAGGNSS
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [AzureKinect, VisionRTK] = readLogPlatform(dataFolder, dataFile, startTime)
% inputs (3) : - dataFolder is the name of the folder where the raw data
%                 are stored, FolderData must be specified as a character
%                 or a string 
%
%              - dataFile is the name of the file where the raw data
%                 are stored, dataFile must be specified as a character
%                 or a string 
%
%              - startTime is the time when the measurements took place,
%                 startTime must be specified as a scalar string
%
% outputs (2): - AzureKinect is a struct regrouping all the information related to
%                the Azure Kinect measurement. (ref. section Software of master thesis)
%
%              - VisionRTK is a struct regrouping all the information related to
%                the vision-RTK measurement. (ref. section Software of master thesis)
%

%Create the path to the files of platform 
rosBagPath = fullfile(dataFolder, dataFile);
varAzurePath = fullfile(dataFolder,['AzureKinect_',startTime,'.mat']);
varVisionPath = fullfile(dataFolder,['VisionRTK_',startTime,'.mat']);

%For computing efficiency reason check if the variables AzureKinect is already saved
if isfile(varAzurePath) 
    load(varAzurePath);
    
else
    %Extract the data corresponding to the TOPIC READBAGIMU(BAG, TOPIC) 
    platformBag = rosbag(rosBagPath);
    AzureKinect.Imu = readBagIMU(platformBag, '/imu');
    save(varAzurePath, 'AzureKinect');
    
end

%For computing efficiency reason check if the variables VisionRTK is already saved
if isfile(varVisionPath)
    load(varVisionPath);

else
    if ~exist('plaftformBag','var')
        platformBag = rosbag(rosBagPath);
    end
    
    %Extract the data corresponding to the TOPIC READBAGIMU(BAG, TOPIC) 
    VisionRTK.Imu = readBagIMU(platformBag, '/imu/data');
    VisionRTK.Gnss2 = readBagGNSS(platformBag, '/gnss1/fix');
    VisionRTK.Gnss1 = readBagGNSS(platformBag, '/gnss2/fix');
    save(varVisionPath, 'VisionRTK');

end 


end