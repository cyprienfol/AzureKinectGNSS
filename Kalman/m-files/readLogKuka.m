% Read Kuka .txt DATAFILE associate to STARTTIME in DATAFOLDER.
%   readLogKuka.m 
%
%   See also FULLFILE, ISFILE, LOAD,  SAVE, matlab built-in function, 
%            PREPROCESSKUKA
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%
function Kuka = readLogKuka(dataFolder, dataFile, startTime)
% inputs (3)  : - dataFile is the name of the file where the raw data
%                 are stored, dataFile must be specified as a character
%                 or a string 
%               
%               - dataFolder is the name of the folder where the raw data
%                 are stored, FolderData must be specified as a character
%                 or a string 
%
%               - startTime is the time when the measurements took place,
%                 startTime must be specified as a scalar string
%
% output (1): - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Chapter 4 Report)         

%For computing efficiency check if Kuka struct is already saved
txtKukaPath = fullfile(dataFolder, dataFile);
varKukaPath = fullfile(dataFolder,['Kuka_',startTime,'.mat']);

if isfile(varKukaPath)
    load(varKukaPath);
   
else
    Kuka.rawdata = load(txtKukaPath);
    [Kuka.timestamp, Kuka.trajectory] = preprocessingKuka(Kuka.rawdata);
    save(varKukaPath, 'Kuka');

end

end