% Synchronize the timeseries TIMESTAMP_1, TIMESTAMP_2  
%   synchroniseMeasurement.m 
%
%   See also LENGTH matlab built-in function, 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.

function [startInd_1, startInd_2] = synchroniseMeasurement(timestamp_1, timestamp_2)
% inputs (2)  : - timestamp_1 is one of the timestep of the timeseries to
%                 synchronized
%
%               - timestamp_2 is the other one timestep of the timeseries to
%                 synchronized
%
% outputs (2) : - startInd_1 is the first index in timestamp_1 sychronized with 
%                 timestamp_2 
%
%               - startInd_2 is the first index in timestamp_2 sychronized with 
%                 timestamp_1 
%


%Initialise the Index for the timeseries
startInd_1 = 1; 
startInd_2 = 1;

%Check which timeseries started first
if (timestamp_2(1,1) > timestamp_1(1,1))
    %Check which timestamp is the closest to the other timeserie
    while  startInd_1 + 1 <= length(timestamp_1)
        if timestamp_1(startInd_1+1,1) < timestamp_2(1,1)
            startInd_1 = startInd_1 + 1;
            
        else
            break;
            
        end
    end
    
else
    %Check which timestamp is the closest to the other timeserie
    while startInd_2 + 1<=length(timestamp_2)
        if timestamp_2(startInd_2+1,1) < timestamp_1(1,1)
            startInd_2 = startInd_2 + 1;
            
        else
            break;            
        
        end
    end
end

if startInd_1 > 1
    startInd_1 = startInd_1 + 1;
end

if startInd_2 > 1
   startInd_2 = startInd_2 + 1;
end

end