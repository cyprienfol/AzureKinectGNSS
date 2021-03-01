% Compare visually the sensor Fusion from Fixposition and Ours with plots, and perform basic statistical analysis
%   compareSensorFusion.m 
%
%   See also TIMESERIES, SYNCHRONIZE, RESAMPLE, PLOT, matlab built-in function
%            COMPARE2D, COMPARE3D
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function compareSensorFusion(Kuka, Fusion, Fixposition)
% inputs (3) : - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Chapter 4 Report)  
%
%              - Fusion is a struct regrouping all the information related to
%                 the sensor Fusion. (ref. Ch.4 of report)
%
%              - Fixposition is a struct regrouping all the information related to
%                the sensor Fusion performed by Fixposition proprietory algorithm 
%   

%Create timeseries in order to compute residuals
ts_kuka = timeseries([Kuka.trajectory.position, Kuka.trajectory.orientation], Kuka.timestamp);

ts_mechanisation = timeseries([Fusion.position', Fusion.attitude'], Fusion.timestamp);

ts_fixposition = timeseries([Fixposition.position', Fixposition.attitude'], Fixposition.timestamp);

%Synchronize data for comparison
[ts_kuka_sync, ts_fixposition_sync] = synchronize(ts_kuka, ts_fixposition, 'Union');
[ts_kuka_sync, ts_mechanisation_sync] = synchronize(ts_kuka_sync, ts_mechanisation, 'Union');

%Subsample the Timeseries
timevec = [ts_kuka_sync.TimeInfo.Start:0.1:ts_kuka_sync.TimeInfo.End];
ts_kuka_sample = resample(ts_kuka,timevec);
ts_mechanisation_sample = resample(ts_mechanisation,timevec);
ts_fixposition_sample = resample(ts_fixposition,timevec);

%% PART I: Plot the Sensor Fusion together with Ground truth
compare2D(ts_kuka_sample , ts_mechanisation_sample  ,ts_fixposition_sample )
compare3D(ts_kuka_sample , ts_mechanisation_sample  ,ts_fixposition_sample )

%% PART II: Compute the residuals and compare it 
%Statistical analysis of the results (Mean, Standard Deviation, Residuals)    
mechanisation_res = ts_mechanisation_sample.Data - ts_kuka_sample.Data;
mechanisation_mean = mean(mechanisation_res);
mechanisation_std = std(mechanisation_res);

fprintf('Mechanisation statistical information: \n');
fprintf('Position: \n');
fprintf('x:\t mean: %f [m] \t standard deviation: %f [m]\n', mechanisation_mean(1) , mechanisation_std(1));
fprintf('y:\t mean: %f [m] \t standard deviation: %f [m]\n', mechanisation_mean(2) , mechanisation_std(2));
fprintf('z:\t mean: %f [m] \t standard deviation: %f [m]\n', mechanisation_mean(3) , mechanisation_std(3));

fprintf('Orientation: \n');
fprintf('roll:\t mean: %f [°] \t standard deviation: %f [°]\n', (180/pi)*mechanisation_mean(4) , (180/pi)*mechanisation_std(4));
fprintf('pitch:\t mean: %f [°] \t standard deviation: %f [°]\n', (180/pi)*mechanisation_mean(5) , (180/pi)*mechanisation_std(5));
fprintf('yaw:\t mean: %f [°] \t standard deviation: %f [°]\n', (180/pi)*mechanisation_mean(6) , (180/pi)*mechanisation_std(6));

%Trajectory accuracy
trajectory2d_mecha_res = sqrt(sum((ts_mechanisation_sample.Data(:,1:2) - ts_kuka_sample.Data(:,1:2)).^2, 2));
trajectory2d_mecha_mean = mean(trajectory2d_mecha_res);
trajectory2d_mecha_std = std(trajectory2d_mecha_res);

trajectory3d_mecha_res = sqrt(sum((ts_mechanisation_sample.Data(:,1:3) - ts_kuka_sample.Data(:,1:3)).^2, 2));
trajectory3d_mecha_mean = mean(trajectory3d_mecha_res);
trajectory3d_mecha_std = std(trajectory3d_mecha_res);

fprintf('2D-trajectory : \n');
fprintf('Position:\t mean: %f [m] \t standard deviation: %f [m]\n', trajectory2d_mecha_mean ,trajectory2d_mecha_std);
fprintf('3D-trajectory : \n');
fprintf('Position:\t mean: %f [m] \t standard deviation: %f [m]\n', trajectory3d_mecha_mean ,trajectory3d_mecha_std);

fixposition_res = ts_fixposition_sample.Data - ts_kuka_sample.Data;
fixposition_mean = mean(fixposition_res);
fixposition_std = std(fixposition_res);

fprintf('\nFixposition statistical information: \n');
fprintf('Position: \n');
fprintf('x:\t mean: %f [m] \t standard deviation: %f [m]\n', fixposition_mean(1) , fixposition_std(1));
fprintf('y:\t mean: %f [m] \t standard deviation: %f [m]\n', fixposition_mean(2) , fixposition_std(2));
fprintf('z:\t mean: %f [m] \t standard deviation: %f [m]\n', fixposition_mean(3) , fixposition_std(3));

fprintf('Orientation: \n');
fprintf('roll:\t mean: %f [°] \t standard deviation: %f [°]\n', (180/pi)*fixposition_mean(4) , (180/pi)*fixposition_std(4));
fprintf('pitch:\t mean: %f [°] \t standard deviation: %f [°]\n', (180/pi)*fixposition_mean(5) , (180/pi)*fixposition_std(5));
fprintf('yaw:\t mean: %f [°] \t standard deviation: %f [°]\n', (180/pi)*fixposition_mean(6) , (180/pi)*fixposition_std(6));

%Trajectory accuracy
trajectory2d_fixp_res = sqrt(sum((ts_fixposition_sample.Data(:,1:2) - ts_kuka_sample.Data(:,1:2)).^2, 2));
trajectory2d_fixp_mean = mean(trajectory2d_fixp_res);
trajectory2d_fixp_std = std(trajectory2d_fixp_res);

trajectory3d_fixp_res = sqrt(sum((ts_fixposition_sample.Data(:,1:3) - ts_kuka_sample.Data(:,1:3)).^2, 2));
trajectory3d_fixp_mean = mean(trajectory3d_fixp_res);
trajectory3d_fixp_std = std(trajectory3d_fixp_res);

fprintf('2D-trajectory : \n');
fprintf('Position:\t mean: %f [m] \t standard deviation: %f [m]\n', trajectory2d_fixp_mean ,trajectory2d_fixp_std);
fprintf('3D-trajectory : \n');
fprintf('Position:\t mean: %f [m] \t standard deviation: %f [m]\n', trajectory3d_fixp_mean ,trajectory3d_fixp_std);

figure
subplot(3,1,1)
plot(ts_mechanisation_sample.Time, 100*mechanisation_res(:,1), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 14)
hold on
plot(ts_fixposition_sample.Time, 100*fixposition_res(:,1), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 14)
ylabel('\Delta X [cm]')
grid on
axis padded
ylim([-4, 4])
yticks(-4:1:4)
set(gca, 'FontSize', 25);

subplot(3,1,2)
plot(ts_mechanisation_sample.Time, 100*mechanisation_res(:,2), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 14)
hold on
plot(ts_fixposition_sample.Time, 100*fixposition_res(:,2), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 14)
ylabel('\Delta Y [cm]')
grid on
axis padded
ylim([-4, 4])
yticks(-4:1:4)
set(gca, 'FontSize', 25);

subplot(3,1,3)
plot(ts_mechanisation_sample.Time, 100*mechanisation_res(:,3), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 14)
hold on
plot(ts_fixposition_sample.Time, 100*fixposition_res(:,3), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 14)
ylabel('\Delta Z [cm]')
legend('K.F Fusion', 'Fixposition')
grid on
axis padded
ylim([-4, 4])
yticks(-4:1:4)
set(gca, 'FontSize', 25);

sgtitle('Position Residuals', 'FontSize', 30)

figure
subplot(3,1,1)
plot(ts_mechanisation_sample.Time, (180/pi)*mechanisation_res(:,4), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 14)
hold on
plot(ts_fixposition_sample.Time, (180/pi)*fixposition_res(:,4), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 14)
ylabel('\Delta Roll [°]')
grid on
axis padded
ylim([-1, 1])
yticks(-1:0.25:1)
set(gca, 'FontSize', 25);

subplot(3,1,2)
plot(ts_mechanisation_sample.Time, (180/pi)*mechanisation_res(:,5), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 14)
hold on
plot(ts_fixposition_sample.Time, (180/pi)*fixposition_res(:,5), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 14)
ylabel('\Delta Pitch [°]')
grid on
axis padded
ylim([-1, 1])
yticks(-1:0.25:1)
set(gca, 'FontSize', 25);

subplot(3,1,3)
plot(ts_mechanisation_sample.Time, (180/pi)*mechanisation_res(:,6), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 14)
hold on
plot(ts_fixposition_sample.Time, (180/pi)*fixposition_res(:,6), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 14)
legend('K.F Fusion', 'Fixposition')
xlabel('Time [sec]')
ylabel('\Delta Yaw [°]')
grid on
axis padded
ylim([-6, 6])
yticks(-6:1.5:6)
set(gca, 'FontSize', 25);

sgtitle('Orientation Residuals', 'FontSize', 30)



end