% Compare visually the AZUREACCEl and VISIONACCEL with plots and perform basic statistical analysis  
%   compareGyro.m 
%
%   See also TIMESERIES, SYNCHRONIZE, PLOT, matlab built-in function
%            PLOTACCEL
%            
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function compareAccel(Kuka, AzureAccel, AzureTime, VisionAccel, VisionTime)
% inputs (5) : - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - AzureAccel is a 3xn double vector that represents the
%                position obtained by integrating the Azure Kinect
%                Accelerometer
%
%              - AzureTime is 1xn double vector that represents each timesteps 
%                the Azure Kinect IMU was taking measurements
%
%              - VisionAccel is a 3xn double vector that represents the
%                position obtained by integrating the Vision-RTK
%                Acceleromete
%
%              - VisionTime is 1xn double vector that represents each timesteps 
%                the Vision-RTK IMU was taking measurements
%

%Create timeseries in order to compute residuals
ts_kuka = timeseries(Kuka.trajectory.position, Kuka.timestamp-Kuka.timestamp(1,1));

ts_azure = timeseries(AzureAccel, AzureTime-Kuka.timestamp(1,1));

ts_vision = timeseries(VisionAccel, VisionTime-Kuka.timestamp(1,1));

%Synchronize data for comparison
[ts_kuka_sync, ts_azure_sync] = synchronize(ts_kuka, ts_azure, 'Uniform','Interval',0.1);
[ts_kuka_sync, ts_vision_sync, ] = synchronize(ts_kuka,ts_vision, 'Uniform','Interval',0.1);

%Visualy the reference trajectory with the Acceleromerer data    
plotAccel(ts_kuka_sync, ts_azure_sync, ts_vision_sync);

%Statistical analysis of the results (Mean, Standard Deviation, Residuals)    
azure_res = ts_azure_sync.Data - ts_kuka_sync.Data;
azure_mean = mean(azure_res);
azure_std = std(azure_res);

fprintf('Vision RTK Accelerometer statistical information: \n');
fprintf('roll:\t mean: %f [rad] \t standard deviation: %f [rad]\n', azure_mean(1) , azure_std(1));
fprintf('pitch:\t mean: %f [rad] \t standard deviation: %f [rad]\n', azure_mean(2) , azure_std(2));
fprintf('yaw:\t mean: %f [rad] \t standard deviation: %f [rad]\n', azure_mean(3) , azure_std(3));

vision_res = ts_vision_sync.Data - ts_kuka_sync.Data;
vision_mean = mean(vision_res);
vision_std = std(vision_res);

fprintf('Vision RTK Accelerometer statistical information: \n');
fprintf('roll:\t mean: %f [rad] \t standard deviation: %f [rad]\n', vision_mean(1) , vision_std(1));
fprintf('pitch:\t mean: %f [rad] \t standard deviation: %f [rad]\n', vision_mean(2) , vision_std(2));
fprintf('yaw:\t mean: %f [rad] \t standard deviation: %f [rad]\n', vision_mean(3) , vision_std(3));


figure
subplot(3,1,1)
plot(ts_azure_sync.Time, azure_res(:,1), '.', 'color', [0.8500, 0.3250, 0.09800])
hold on
plot(ts_vision_sync.Time, vision_res(:,1), '.', 'color', [0.9290, 0.6940, 0.1250])
axis padded
xlabel('Time [sec]')
ylabel('Devaition [m]')
grid on
legend('Azure Kinect', 'Vision RTK', 'Location', 'southeastoutside')
title('X: residuals');

subplot(3,1,2)
plot(ts_azure_sync.Time, azure_res(:,2), '.', 'color', [0.8500, 0.3250, 0.09800])
hold on
plot(ts_vision_sync.Time, vision_res(:,2), '.', 'color', [0.9290, 0.6940, 0.1250])
axis padded
xlabel('Time [m]')
ylabel('Devaition [rad]')
grid on
legend('Azure Kinect', 'Vision RTK', 'location', 'southeastoutside')
title('Y: residuals');

subplot(3,1,3)
plot(ts_azure_sync.Time, azure_res(:,3), '.', 'color', [0.8500, 0.3250, 0.09800])
hold on
plot(ts_vision_sync.Time, vision_res(:,3), '.', 'color', [0.9290, 0.6940, 0.1250])
axis padded
xlabel('Time [sec]')
ylabel('Devaition [m]')
grid on
legend('Azure Kinect', 'Vision RTK', 'location', 'southeastoutside')
title('Z: residuals');



end