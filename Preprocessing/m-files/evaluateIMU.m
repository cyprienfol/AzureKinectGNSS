% Evaluate the IMU the coordinate transformation on the SYNCIMU
%   evaluateIMU.m 
%
%   See also PLOT, matlab built-in function
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%


function evaluateIMU(syncImu, Imu, name)
% inputs (3) : - syncImu is a struct that regroups the the IMU measurement 
%                after synchronisation 
%
%              - Imu is a struct that regroup all the information from
%                IMU sensor before synchronisation
%
%              - name is a chain of character to determine which IMU of the platform 
%                is evaluated (either Azure Kinect or Vision-RTK)     
%
switch name
    case 'vision'
        colorImu  = [0.9290, 0.6940, 0.1250];
    case 'azure'
        colorImu  = [0.8500, 0.3250, 0.09800];
    otherwise
        warning('The default color is chosen')
        colorImu = [0, 0, 0];
end

%Plot the integrated position in before and after Transformation
figure
subplot(2,3,1)
plot(syncImu.timestamp(3:end)-syncImu.timestamp(1), syncImu.integration.position(:, 1), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('X [m]')
set(gca, 'FontSize', 25)

subplot(2,3,2)
plot(syncImu.timestamp(3:end)-syncImu.timestamp(1), syncImu.integration.position(:, 2), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Y [m]')
set(gca, 'FontSize', 25)
title('Body C.S', 'FontSize', 30)

subplot(2,3,3)
plot(syncImu.timestamp(3:end)-syncImu.timestamp(1), syncImu.integration.position(:, 3), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Z [m]')
set(gca, 'FontSize', 25)

subplot(2,3,4)
plot(Imu.timestamp(3:end)-Imu.timestamp(1), Imu.integration.position(:, 1), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('X [m]')
xlabel('Time [s]')
set(gca, 'FontSize', 25)


subplot(2,3,5)
plot(Imu.timestamp(3:end)-Imu.timestamp(1), Imu.integration.position(:, 2), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Y [m]')
xlabel('Time [s]')
set(gca, 'FontSize', 25)
title('Local C.S', 'FontSize', 30)

subplot(2,3,6)
plot(Imu.timestamp(3:end)-Imu.timestamp(1), Imu.integration.position(:, 3), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Z [m]')
xlabel('Time [s]')
set(gca, 'FontSize', 25)

%Plot the integrated orientation

figure
subplot(2,3,1)
plot(syncImu.timestamp-syncImu.timestamp(1), syncImu.integration.attitude(:, 1), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Roll [°]')
set(gca, 'FontSize', 25)

subplot(2,3,2)
plot(syncImu.timestamp-syncImu.timestamp(1), syncImu.integration.attitude(:, 2), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Pitch [°]')
set(gca, 'FontSize', 25)
title('Body C.S', 'FontSize', 30)

subplot(2,3,3)
plot(syncImu.timestamp-syncImu.timestamp(1), syncImu.integration.attitude(:, 3), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Yaw [°]')
set(gca, 'FontSize', 25)

subplot(2,3,4)
plot(Imu.timestamp-Imu.timestamp(1), Imu.integration.attitude(:, 1), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Roll [°]')
xlabel('Time [s]')
set(gca, 'FontSize', 25)


subplot(2,3,5)
plot(Imu.timestamp-Imu.timestamp(1), Imu.integration.attitude(:, 2), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Pitch [°]')
xlabel('Time [s]')
set(gca, 'FontSize', 25)
title('Local C.S', 'FontSize', 30)

subplot(2,3,6)
plot(Imu.timestamp-Imu.timestamp(1), Imu.integration.attitude(:, 3), '.', 'MarkerSize', 14, 'color', colorImu);
grid on;
ylabel('Yaw [°]')
xlabel('Time [s]')
set(gca, 'FontSize', 25)




end