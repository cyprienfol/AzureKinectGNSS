% Synchronize the two timesteps of the GNSS receivers
%   synchroniseGNSS.m 
%
%   See also SYNCHRONISEMEASUREMENT   
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.

function [startInd_1, endInd_1, startInd_2, endInd_2] = synchroniseGnss(timeGnss1_, timeGnss2_)
% inputs (2)  : - timeGNSS1_ is the timesteps vectors of the measurement
%                 time from the left GNSS receiver
%
%               - timeGNSS2_ is the timesteps vectors of the measurement
%                 time from the right GNSS receiver
%
% outputs (4) : - startInd_1 is the index of the left GNSS receiver first measurement synchronized with 
%                 the right GNSS receiver measurement.
%
%               - endInd_1 is the index of the left GNSS receiver last measurement synchronized with 
%                 the right GNSS receiver measurement.
%
%               - startInd_2 is the index of the right GNSS receiver first measurement synchronized with 
%                 the left GNSS receiver measurement.
%
%               - endInd_2 is the index of the right GNSS receiver last measurement synchronized with 
%                 the left GNSS receiver measurement.
%

% Check which Gnss receiver start recording the latest
[startInd_1, startInd_2] = synchroniseMeasurement(timeGnss1_,timeGnss2_);

%Assumption both Gnss receivers have the same sampling frequency
timeGnss1 = timeGnss1_(startInd_1:end);
timeGnss2 = timeGnss2_(startInd_2:end);

if length(timeGnss1) > length(timeGnss2)
    endInd_1 = startInd_1 + length(timeGnss2);
    endInd_2 = startInd_2 + length(timeGnss2);
    
else 
    endInd_1 = startInd_1 + (length(timeGnss1)-1);
    endInd_2 = startInd_2 + (length(timeGnss1)-1);
    
end

end