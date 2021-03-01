% Simulate a laser at the FUSE position steering at TARGET 
%   simulateLaser.m 
%
%   See also TIMESERIES, SYNCHRONIZE matlab built-in function, 
%            line_plane_intersection Author & support : nicolas.douillet (at) free.fr, 2019-2020.
%            ESTIMATEANLGLECORRECTION, VISUALISELASER, VISUALISETARGET
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [intersectionFuse, correctionFuse] = simulateLaser(Fusion, Kuka, Target)
% inputs (3) :  - Fusion is a struct that regroups all the information 
%                 relative to the proprietary Fusion algorithm of Fixposition
%
%               - Kuka is a struct regrouping all the information related to
%                 the Kuka robotic arm measurement. (ref. Chapter 4 Report)    
%
%               - Target is a 3x1 vector of double that represent the position of the 
%                 center of the target aiming by the simulated laser beam
%
% outputs (2)  : - intersectionFuse is a vector of double that represents all
%                 intersections between the laser beam simulated and a imaginery vertical plane at a certain distance
%                 based on the Fixposition navigationnal information 
%
%               - correctionFuse is a 3x1 vector representing the angle to rotate
%                 the platform to steer in direction of the target
%

%Rotation Matrix initialisation
Rx = @(Phi)[ 1 0 0;...
                0 cos(Phi), sin(Phi);...
                0, -sin(Phi), cos(Phi)];
            
Ry = @(Theta)[cos(Theta), 0, -sin(Theta);...
                 0, 1, 0; ...
                 sin(Theta), 0, cos(Theta)];

Rz = @(Psi)[cos(Psi), sin(Psi), 0;...
               -sin(Psi), cos(Psi), 0;...
               0, 0, 1];

%heading initiailisation
initialheading = [1;0;0];

%Subsample the Fuse dataset and the Ground truth
%Create timeseries in order to compute residuals
ts_fuse = timeseries([Fusion.position', Fusion.attitude'],Fusion.timestamp);

ts_kuka = timeseries([Kuka.trajectory.position, Kuka.trajectory.orientation],Kuka.timestamp);

%Synchronize data for comparison
[ts_sync_kuka, ts_sync_fuse] = synchronize(ts_kuka, ts_fuse, 'Uniform','Interval',0.1);

for ind_cor = 1:length(ts_sync_fuse.Time)
    %Estimate the orientation based on the position and attitude of Kuka
    positionKuka(:,ind_cor) = ts_sync_kuka.Data(ind_cor, 1:3)';
    attitudeKuka(:,ind_cor) = ts_sync_kuka.Data(ind_cor, 4:6)';
    headingKuka(:,ind_cor) = Rx(attitudeKuka(1,ind_cor))*Ry(attitudeKuka(2, ind_cor))*Rz(attitudeKuka(3, ind_cor) )*initialheading;
    
    correctionKuka(:,ind_cor) = estimateAngleCorrection(positionKuka(:,ind_cor), attitudeKuka(:,ind_cor), initialheading, Target.position);
    laserKuka = Rx(correctionKuka(1,ind_cor))*Ry(correctionKuka(2, ind_cor))*Rz(correctionKuka(3, ind_cor) )*headingKuka(:,ind_cor);     

    [intersectionKuka(:,ind_cor), rc] = line_plane_intersection(laserKuka, positionKuka(:,ind_cor), Target.normal,Target.position, 1);

    %Estimate the orientation based on the position and attitude of Fusion
    positionFuse(:,ind_cor) = ts_sync_fuse.Data(ind_cor, 1:3)';
    attitudeFuse(:,ind_cor) = ts_sync_fuse.Data(ind_cor, 4:6)';
    
    correctionFuse(:,ind_cor) = estimateAngleCorrection(positionFuse(:,ind_cor), attitudeFuse(:,ind_cor), initialheading, Target.position);
    laserFuse = Rx(correctionFuse(1,ind_cor))*Ry(correctionFuse(2, ind_cor))*Rz(correctionFuse(3, ind_cor) )*headingKuka(:,ind_cor);     
    
    [intersectionFuse(:,ind_cor), rc] = line_plane_intersection(laserFuse, positionKuka(:,ind_cor), Target.normal,Target.position, 1);

    
end

%Visualise the laser
visualiseLaser(positionKuka, attitudeKuka, correctionKuka, correctionFuse, initialheading, Target);

%Calculate the intersection of the laser with the Target plane 
visualiseTarget(intersectionKuka, intersectionFuse, Target);



end