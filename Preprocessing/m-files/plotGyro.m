% Plot the Gyroscope of vision-RTK and Azure Kinect data 
%   plotGyro.m 
%
%   See also PLOT, matlab built-in function
%            
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function plotGyro(ts_kuka, ts_azure, ts_vision)
% inputs (3) : - ts_kuka is a timeseries struct that represents the
%                measurement from Ground truth (Kuka)
%
%              - ts_azure is a timeseries struct that represents the
%                integrated measurement from the Gyroscope on AzureKinect
%
%              - ts_vision is a timeseries struct that represents the
%                integrated measurement from the Gyroscope on
%                Vision-RTK
%
%Compare AZURE with Groundtuth
figure;
subplot(2,3,1);
plot(ts_kuka.Time, (180/pi)*(ts_kuka.Data(:,1)-mean(ts_kuka.Data(:,1))), '.k', 'MarkerSize', 10); 
axis padded
xlabel('Time [s]');
ylabel('Roll [°]');
grid on
set(gca, 'fontsize',16)

subplot(2,3,2);
plot(ts_kuka.Time, (180/pi)*(ts_kuka.Data(:,2)-mean(ts_kuka.Data(:,2))), '.k', 'MarkerSize', 10); 
axis padded
xlabel('Time [s]');
ylabel('Pitch[°]');
grid on
title('Kuka Robot')
set(gca, 'fontsize',16)

subplot(2,3,3);
plot(ts_kuka.Time, (180/pi)*(ts_kuka.Data(:,3)-mean(ts_kuka.Data(:,3))), '.k', 'MarkerSize', 10); 
axis padded
xlabel('Time [s]');
ylabel('Yaw [°]');
grid on
set(gca, 'fontsize',16)

subplot(2,3,4);
plot(ts_azure.Time, (180/pi)*ts_azure.Data(:,1), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 10);
axis padded
xlabel('Time [s]');
ylabel('Roll [°]');
grid on
set(gca, 'fontsize',16)

subplot(2,3,5);
plot(ts_azure.Time, (180/pi)*ts_azure.Data(:,2), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 10);
axis padded
xlabel('Time [s]');
ylabel('Pitch [°]');
grid on
title('Azure Kinect')
set(gca, 'fontsize',16)

subplot(2,3,6);
plot(ts_azure.Time, (180/pi)*ts_azure.Data(:,3), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 10);
axis padded
xlabel('Time [s]');
ylabel('Yaw [°]');
grid on
set(gca, 'fontsize',16)


%Compare Vision-RTK with Groundtuth
figure;
subplot(2,3,1);
plot(ts_kuka.Time, (180/pi)*(ts_kuka.Data(:,1)-mean(ts_kuka.Data(:,1))), '.k', 'MarkerSize', 10); 
axis padded
xlabel('Time [s]');
ylabel('Roll [°]');
grid on
set(gca, 'fontsize',16)

subplot(2,3,2);
plot(ts_kuka.Time, (180/pi)*(ts_kuka.Data(:,2)-mean(ts_kuka.Data(:,2))), '.k', 'MarkerSize', 10); 
axis padded
xlabel('Time [s]');
ylabel('Pitch [°]');
grid on
title('Kuka Robot')
set(gca, 'fontsize',16)

subplot(2,3,3);
plot(ts_kuka.Time, (180/pi)*(ts_kuka.Data(:,3)-mean(ts_kuka.Data(:,3))), '.k', 'MarkerSize', 10); 
axis padded
xlabel('Time [s]');
ylabel('Yaw [°]');
grid on
set(gca, 'fontsize',16)

subplot(2,3,4);
plot(ts_vision.Time, (180/pi)*ts_vision.Data(:,1), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 10);
axis padded
xlabel('Time [s]');
ylabel('Roll [°]');
grid on
set(gca, 'fontsize',16)

subplot(2,3,5);
plot(ts_vision.Time,(180/pi)* ts_vision.Data(:,2), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 10);
axis padded
xlabel('Time [s]');
ylabel('Pitch [°]');
grid on
title('Vision-RTK')
set(gca, 'fontsize',16)

subplot(2,3,6);
plot(ts_vision.Time, (180/pi)*ts_vision.Data(:,3), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 10);
axis padded
xlabel('Time [s]');
ylabel('Yaw [°]');
grid on
set(gca, 'fontsize',16)

%Plot the value of the integrated angular velocity of Platform
figure;
subplot(3,1,1);
plot(ts_kuka.Time, (180/pi)*ts_kuka.Data(:,1), '.k', 'MarkerSize', 12); 
hold on
plot(ts_azure.Time, (180/pi)*ts_azure.Data(:,1), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 12);
plot(ts_vision.Time, (180/pi)*ts_vision.Data(:,1), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 12);
axis padded
ylabel('Roll [°]');
grid on
legend('Ground Truth', 'Azure Kinect', 'Vision RTK', 'location', 'southeastoutside');
title('Orientation')
set(gca, 'Fontsize', 25);

subplot(3,1,2);
plot(ts_kuka.Time, (180/pi)*ts_kuka.Data(:,2), '.k', 'MarkerSize', 12); 
hold on
plot(ts_azure.Time, (180/pi)*ts_azure.Data(:,2), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 12);
plot(ts_vision.Time,(180/pi)* ts_vision.Data(:,2), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 12);
axis padded
ylabel('Pitch [°]');
grid on
legend('Ground Truth', 'Azure Kinect', 'Vision RTK', 'location', 'southeastoutside');
set(gca, 'Fontsize', 25);

subplot(3,1,3);
plot(ts_kuka.Time, (180/pi)*ts_kuka.Data(:,3), '.k', 'MarkerSize', 12); 
hold on
plot(ts_azure.Time, (180/pi)*ts_azure.Data(:,3), '.', 'color', [0.8500, 0.3250, 0.09800], 'MarkerSize', 12);
plot(ts_vision.Time, (180/pi)*ts_vision.Data(:,3), '.', 'color', [0.9290, 0.6940, 0.1250], 'MarkerSize', 12);
axis padded
xlabel('Time [s]');
ylabel('Yaw [°]');
grid on
legend('Ground Truth', 'Azure Kinect', 'Vision RTK', 'location', 'southeastoutside');
set(gca, 'Fontsize', 25);


end