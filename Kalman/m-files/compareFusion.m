% Compare visually the FUSION and KUKA trajectories with plots, and perform basic statistical analysis
%   compareFusion.m 
%
%   See also TIMESERIES, SYNCHRONIZE, RESAMPLE, PLOT, matlab built-in function
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function compareFusion(Fusion, Kuka, Parameter)
% inputs (3) : - Fusion is a struct regrouping all the information obtained
%                after the Sensor Fusion. (ref. Ch.4 of report)
%
%              - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - Parameter contains the information influencing
%                the sensor fusion based on the experiment performed
%              

%Plot trajectory in their own reference frames
figure
subplot(3,1,1)
plot(Fusion.timestamp-Kuka.timestamp(1,1), Fusion.position(1,:), '.g', 'Markersize', 14)
hold on;
plot(Kuka.timestamp-Kuka.timestamp(1,1), Kuka.trajectory.position(:,1), '.k', 'Markersize', 8)
grid on
ylabel('X [m]');
ylim([-0.5, 0.5])
axis padded
set(gca,'Fontsize',25);

subplot(3,1,2)
plot(Fusion.timestamp-Kuka.timestamp(1,1), Fusion.position(2,:), '.g', 'Markersize', 14)
hold on;
plot(Kuka.timestamp-Kuka.timestamp(1,1), Kuka.trajectory.position(:,2),'.k', 'Markersize', 8)
grid on
ylabel('Y [m]');
ylim([-0.5, 0.5])
axis padded
set(gca,'Fontsize',25);

subplot(3,1,3)
plot(Fusion.timestamp-Kuka.timestamp(1,1), Fusion.position(3,:), '.g', 'Markersize', 14)
hold on;
plot(Kuka.timestamp-Kuka.timestamp(1,1), Kuka.trajectory.position(:,3), '.k', 'Markersize', 8)
grid on
xlabel('Time [s]');
ylabel('Z [m]');
legend('K.F Fusion','Kuka Reference','location', 'southeastoutside')
axis padded
ylim([-0.05, 0.05])
set(gca,'Fontsize',25);

sgtitle('Position', 'FontSize', 30)

figure
subplot(3,1,1)
plot(Kuka.timestamp-Kuka.timestamp(1,1), (180/pi)*Kuka.trajectory.orientation(:,1), '.k', 'Markersize', 14)
hold on;
plot(Fusion.timestamp-Kuka.timestamp(1,1), (180/pi)*Fusion.attitude(1,:), '.g', 'Markersize', 8)
grid on
ylabel('Roll [°]');
axis padded
ylim([-0.2, 0.2])
yticks(-0.2:0.05:0.2);
set(gca,'Fontsize',25);

subplot(3,1,2)
plot(Kuka.timestamp-Kuka.timestamp(1,1), (180/pi)*Kuka.trajectory.orientation(:,2),'.k', 'Markersize', 14)
hold on;
plot(Fusion.timestamp-Kuka.timestamp(1,1), (180/pi)*Fusion.attitude(2,:), '.g', 'Markersize', 8)
grid on
ylabel('Pitch[°]');
axis padded
ylim([-0.2, 0.2])
yticks(-0.2:0.05:0.2);
set(gca,'Fontsize',25);

subplot(3,1,3)
plot(Fusion.timestamp-Kuka.timestamp(1,1), (180/pi)*Fusion.attitude(3,:), '.g', 'Markersize', 14)
hold on;
plot(Kuka.timestamp-Kuka.timestamp(1,1), (180/pi)*Kuka.trajectory.orientation(:,3), '.k', 'Markersize', 8)
grid on
xlabel('Time [s]');
ylabel('Yaw [°]');
legend('K.F Fusion','Kuka Reference','location', 'southeastoutside')
axis padded
set(gca,'Fontsize',25);

sgtitle('Orientation', 'FontSize', 30)


%Plot two trajectory in the local frame
figure
hold on;plot(Kuka.trajectory.position(:,1), Kuka.trajectory.position(:,2), 'og')
plot(Fusion.position(1,:), Fusion.position(2,:), '.r','markersize', 20);
plot(Fusion.measurement(1,:), Fusion.measurement(2,:), 'xb', 'markersize', 20);
grid on
title('2D Trajectory')

%% Residuals Analysis
%Create timeseries in order to compute residuals
time_fuse = Fusion.timestamp-Kuka.timestamp(1,1);
ts_fuse = timeseries([Fusion.position', Fusion.attitude'],time_fuse);

time_kuka = Kuka.timestamp-Kuka.timestamp(1,1);
ts_kuka = timeseries([Kuka.trajectory.position, Kuka.trajectory.orientation],time_kuka);

%Synchronize data for comparison
[ts_kuka_sync, ts_fuse_sync] = synchronize(ts_kuka, ts_fuse,'Union');

%Subsample the Timeseries
timevec = [ts_kuka_sync.TimeInfo.Start:0.1:ts_kuka_sync.TimeInfo.End];
ts_kuka_sample = resample(ts_kuka,timevec);
ts_fuse_sample = resample(ts_fuse_sync,timevec);

%Calculate the residuals
fuse_res = ts_fuse_sample.Data - ts_kuka_sample.Data;
fuse_mean = mean(fuse_res);
fuse_std = std(fuse_res);

fprintf('Sensor Fusion statistical information: \n');
fprintf('Position: \n');
fprintf('x:\t mean: %f [m] \t standard deviation: %f [m]\n', fuse_mean(1) , fuse_std(1));
fprintf('y:\t mean: %f [m] \t standard deviation: %f [m]\n', fuse_mean(2) , fuse_std(2));
fprintf('z:\t mean: %f [m] \t standard deviation: %f [m]\n', fuse_mean(3) , fuse_std(3));

fprintf('Orientation: \n');
fprintf('roll:\t mean: %f [°] \t standard deviation: %f [°]\n', (180/pi)*fuse_mean(4) , (180/pi)*fuse_std(4));
fprintf('pitch:\t mean: %f [°] \t standard deviation: %f [°]\n', (180/pi)*fuse_mean(5) , (180/pi)*fuse_std(5));
fprintf('yaw:\t mean: %f [°] \t standard deviation: %f [°]\n', (180/pi)*fuse_mean(6) , (180/pi)*fuse_std(6));

%Trajectory accuracy
trajectory2d_fuse_res = sqrt(sum((ts_fuse_sample.Data(:,1:2) - ts_kuka_sample.Data(:,1:2)).^2, 2));
trajectory2d_fuse_mean = mean(trajectory2d_fuse_res);
trajectory2d_fuse_std = std(trajectory2d_fuse_res);

trajectory3d_fuse_res = sqrt(sum((ts_fuse_sample.Data(:,1:3) - ts_kuka_sample.Data(:,1:3)).^2, 2));
trajectory3d_fuse_mean = mean(trajectory3d_fuse_res);
trajectory3d_fuse_std = std(trajectory3d_fuse_res);

fprintf('2D-trajectory : \n');
fprintf('Position:\t mean: %f [m] \t standard deviation: %f [m]\n', trajectory2d_fuse_mean ,trajectory2d_fuse_std);
fprintf('3D-trajectory : \n');
fprintf('Position:\t mean: %f [m] \t standard deviation: %f [m]\n', trajectory3d_fuse_mean ,trajectory3d_fuse_std);

%Plot the residuals 
figure
subplot(3,1,1)
plot(timevec, 100*fuse_res(:,1), '.', 'color', [1 0.2 0.2], 'MarkerSize', 14);
ylabel('\Delta X [cm]')
grid on
axis padded
ylim([-3, 3])
yticks(-3:1:3)
set(gca,'Fontsize',25);

subplot(3,1,2)
plot(timevec, 100*fuse_res(:,2), '.', 'color', [1 0.2 0.2], 'MarkerSize', 14);
ylabel('\Delta Y [cm]')
grid on
axis padded
ylim([-3, 3])
yticks(-3:1:3)
set(gca,'Fontsize',25);

subplot(3,1,3)
plot(timevec, 100*fuse_res(:,3),'.', 'color', [1 0.2 0.2], 'MarkerSize', 14);
xlabel('time [sec]')
ylabel('\Delta Z [cm]')
grid on
axis padded
ylim([-3, 3])
yticks(-3:1:3)
set(gca,'Fontsize',25);

sgtitle('Position Residuals', 'FontSize', 30)

figure
subplot(3,1,1)
plot(timevec, (180/pi)*fuse_res(:,4), '.', 'color', [1 0.2 0.2], 'MarkerSize', 14);
ylabel('\Delta Roll [°]')
grid on
axis padded
ylim([-0.15, 0.15])
yticks(-0.15:0.05:0.15);
set(gca,'Fontsize',25);

subplot(3,1,2)
plot(timevec, (180/pi)*fuse_res(:,5), '.', 'color', [1 0.2 0.2], 'MarkerSize', 14);
ylabel('\Delta Pitch [°]')
grid on
axis padded
ylim([-0.15, 0.15])
yticks(-0.15:0.05:0.15);
set(gca,'Fontsize',25);

subplot(3,1,3)
plot(timevec, (180/pi)*fuse_res(:,6),'.', 'color', [1 0.2 0.2], 'MarkerSize', 14);
xlabel('time [sec]')
ylabel('\Delta Yaw [°]')
grid on
axis padded
ylim([-6, 6])
yticks(-6:1.5:6);
set(gca,'Fontsize',25);

sgtitle('Orientation Residuals', 'FontSize', 30)

end