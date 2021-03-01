% Estimate the timeshift and the offset between measurements and ground
% truth
%   estimateParameter.m 
%
%   See also MEDIAN, matlab built-in function
%            CONVERTGEODETICTOCARTESIAN
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.

function parameter = estimateParameter(VisionRTK, nameMotion, startTime)
% input (2)  : - VisionRTK is a struct regrouping all the information related to
%                the vision-RTK measurement. (ref. section Software of master thesis)
%
%              - nameMotion is the type of motion performed while the experiment
%                nameMotion must be specified as string or character.
%
%              - startTime is the time when the measurements took place,
%                startTime must be specified as a scalar string
%
% output (1): - parameter is a struct that contains the information influencing
%                the sensor fusion based on the experiment performed
% 

switch nameMotion
    case 'Linear'
        %Linear Motion implies constant velocity and no acceleration
        parameter.imuflag = false;
        parameter.gyroflag = true;
        
        %The platform is at rest for the 50 first measurements
        restPeriode = 50;
                
        %Correct for the timeshift between the ground truth and measurements  
        if startTime == '2020_11_05_1545'
           parameter.timeshift = 13.1;%timeKuka(1627) - timeImu(22921);

        elseif startTime == '2020_11_05_1620'
           parameter.timeshift =    7.278010;%timeKuka(835) - timeGNSS(70);

        else
           Warning('Start Time for linear motion incorrect')
        end
    
    case 'Infinite'
        %During scanning motion the angular velocity was null
        parameter.imuflag = true;
        parameter.gyroflag = false;  
        
        %The platform is at rest for the 50 first measurements
        restPeriode = 50;
        
        %Correct for the timeshift between the ground truth and measurements        
        if startTime == '2020_11_05_1555'
           parameter.timeshift = 0; %timeKuka(835) - timeImu(18519);

        elseif startTime == '2020_11_05_1625'
           parameter.timeshift = 6.390789;% timeKuka(835) - timeImu(5408);

        else
           Warning('Start Time for linear motion incorrect')
        end

    case 'Scanning'
        parameter.imuflag = true;
        parameter.gyroflag = true;  
        
        %The platform is at rest for the 50 first measurements
        restPeriode = 50;
        
        %Correct for the timeshift between the ground truth and measurements  
        if startTime == '2020_11_05_1550'
           parameter.timeshift = 0;%timeKukap(835) - timeImu(18519);

        elseif startTime == '2020_11_05_1615'
           parameter.timeshift = 6.492713;%timeKuka(835) - timeImu(6000);

        else
           Warning('Start Time for linear motion incorrect')
        end
        
    case 'Tracking'
        parameter.imuflag = true;
        parameter.gyroflag = true; 
        
        %The platform is at rest for the 50 first measurements
        restPeriode = 50;
        
        %TO DO ONCE THE MOTION WOULD RECORDED
%         if startTime == '2020_11_05_1545'
%            parameter.timeshift = Kuka.timestamp(1627) - Fuse.timestamp(22921);
% 
%         elseif startTime == '2020_11_05_1620'
%            paramter.timeshift = Kuka.timestamp(1627) - Fuse.timestamp(22921);
% 
%         else
%            Warning('Start Time for linear motion incorrect')
%         end
    otherwise
        error('Enter a valid Trajectory Folder');
        
end
parameter.gnss1refpoint = estimateRefPoint(VisionRTK.Gnss1,restPeriode);
parameter.gnss2refpoint = estimateRefPoint(VisionRTK.Gnss2,restPeriode);

%Get the center of the Gnss baseline
parameter.center.latitude = 0.5*(parameter.gnss1refpoint.latitude + parameter.gnss2refpoint.latitude);
parameter.center.longitude = 0.5*(parameter.gnss1refpoint.longitude + parameter.gnss2refpoint.longitude);
parameter.center.altitude = 0.5*(parameter.gnss1refpoint.altitude + parameter.gnss2refpoint.altitude);
parameter.center.xyz = 0.5*(parameter.gnss1refpoint.xyz + parameter.gnss2refpoint.xyz);

end
