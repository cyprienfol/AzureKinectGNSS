% Read Sensors bag. DATAFILE associated to STARTTIME in DATAFOLDER 
%   readLogVisionRTK.m 
%
%   See also FULLFILE, ISFILE, LOAD, SAVE matlab built-in function, 
%            ROSBAG matlab ROS TOOLBOX
%            READBAGIMU, READBAGGNSS, READBAGFUSION
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function VisionRTK = readLogVisionRTK(dataFolder, dataFile, startTime)
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
% output (1): - VisionRTK is a struct regrouping all the information related to
%                the vision-RTK measurement. (ref. section Software of master thesis)
%

%Create the path to the files of platform 
rosBagPath = fullfile(dataFolder, dataFile);
varVisionPath = fullfile(dataFolder,['VisionRTK_',startTime,'.mat']);

%For computing efficiency reason check if the variable visionRTK is saved
if isfile(varVisionPath)
    load(varVisionPath);

else

    platformBag = rosbag(rosBagPath);
    VisionRTK.Imu = readBagIMU(platformBag, '/imu/data');
    VisionRTK.Gnss2 = readBagGNSS(platformBag, '/gnss1/fix');
    VisionRTK.Gnss1 = readBagGNSS(platformBag, '/gnss2/fix');
    VisionRTK.Fusion = readBagFusion(platformBag, '/vio/odometry');
    save(varVisionPath, 'VisionRTK');

end 


end