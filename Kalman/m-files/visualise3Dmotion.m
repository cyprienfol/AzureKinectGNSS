% Visualise position and orientation with plots in 3D, the platform and the
% ground truth
%   visualise3Dmotion.m 
%
%   See also TIMESERIES, SYNCHRONIZE, QUIVER3, matlab built-in function
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function visualise3Dmotion(Kuka, Fusion)
% inputs (2) : - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - Fusion is a struct regrouping all the information related to
%                the sensor Fusion. (ref. Ch.4 of report)
%  

%Initialize the rotation matrix counter clockwise
Rx = @(Phi)[ 1 0 0; 0 cos(Phi), sin(Phi); 0, -sin(Phi), cos(Phi)];
Ry = @(Theta)[cos(Theta), 0, -sin(Theta); 0, 1, 0; sin(Theta), 0, cos(Theta)];
Rz = @(Psi)[cos(Psi), sin(Psi), 0; -sin(Psi), cos(Psi), 0; 0, 0, 1];

%Create the unit axis coordinates on motion
ex = 0.01*[1;0;0];
ey = 0.01*[0;1;0];
ez = 0.01*[0;0;1];

%Subsample trajectory for better visibility on the graph
timeSubsample = 1:5600:length(Fusion.timestamp);

%Create timeseries in order to compute residuals
time_fuse = Fusion.timestamp(timeSubsample)-Kuka.timestamp(1,1);
ts_fuse = timeseries([Fusion.position(:,timeSubsample)', Fusion.attitude(:,timeSubsample)'],time_fuse());

time_kuka = Kuka.timestamp-Kuka.timestamp(1,1);
ts_kuka = timeseries([Kuka.trajectory.position, Kuka.trajectory.orientation],time_kuka);

%Synchronize data for comparison
[ts_sync_kuka, ts_sync_fuse] = synchronize(ts_kuka, ts_fuse, 'Union')

for ind = 1:16000:length(Fusion.timestamp)
    %Update location of the axis based on the platform
    ex_fuse(:,iter_fuse) = Rx(Fusion.attitude(1,ind))*Ry(Fusion.attitude(2,ind))*Rz(Fusion.attitude(3,ind))*ex; 
    ey_fuse(:,iter_fuse) = Rx(Fusion.attitude(1,ind))*Ry(Fusion.attitude(2,ind))*Rz(Fusion.attitude(3,ind))*ey; 
    ez_fuse(:,iter_fuse) = Rx(Fusion.attitude(1,ind))*Ry(Fusion.attitude(2,ind))*Rz(Fusion.attitude(3,ind))*ez;  
    iter_fuse = iter_fuse + 1;
    
end 

%Generate Reference axis
Kuka_sample = Kuka.trajectory.position(1:1000:end,:)';%100[Hz]-> 0.1[Hz] 
iter_kuka = 1;
for ind = 1:1000:length(Kuka.timestamp)
    %Update location of the axis based on the platform
    ex_kuka(:,iter_kuka) = Rx(Fusion.attitude(1,ind))*Ry(Fusion.attitude(2,ind))*Rz(Fusion.attitude(3,ind))*ex; 
    ey_kuka(:,iter_kuka) = Rx(Fusion.attitude(1,ind))*Ry(Fusion.attitude(2,ind))*Rz(Fusion.attitude(3,ind))*ey; 
    ez_kuka(:,iter_kuka) = Rx(Fusion.attitude(1,ind))*Ry(Fusion.attitude(2,ind))*Rz(Fusion.attitude(3,ind))*ez;   
    iter_kuka = iter_kuka + 1;
end 

%Plot the trajectory every 10 second with the platform orientations
figure
%Plot the x axis
quiver3(Fuse_sample(1,:), Fuse_sample(2,:), Fuse_sample(3,:),ex_fuse(1,:),ex_fuse(2,:),ex_fuse(3,:));
hold on;
%PLot the y axis
quiver3(Fuse_sample(1,:), Fuse_sample(2,:), Fuse_sample(3,:),ey_fuse(1,:),ey_fuse(2,:),ey_fuse(3,:));
%Plot the z axis
quiver3(Fuse_sample(1,:), Fuse_sample(2,:), Fuse_sample(3,:),ez_fuse(1,:),ez_fuse(2,:),ez_fuse(3,:));

%Plot the x axis
quiver3(Kuka_sample(1,:), Kuka_sample(2,:), Kuka_sample(3,:),ex_kuka(1,:),ex_kuka(2,:),ex_kuka(3,:));
hold on;
%PLot the y axis
quiver3(Kuka_sample(1,:), Kuka_sample(2,:), Kuka_sample(3,:),ey_kuka(1,:),ey_kuka(2,:),ey_kuka(3,:));
%Plot the z axis
quiver3(Kuka_sample(1,:), Kuka_sample(2,:), Kuka_sample(3,:),ez_kuka(1,:),ez_kuka(2,:),ez_kuka(3,:));

end