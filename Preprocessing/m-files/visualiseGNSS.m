% Visualise the transformation applied to GNSS measurements
%   visualiseGNSS.m 
%
%   See also PLOT, matlab built-in function, 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function visualiseGNSS(VisionRTK) 
% input(1) : - VisionRTK is a struct regrouping all the information related to
%              the vision-RTK measurement. (ref. Ch.4 of report)
%

%Convert UNIX time into local time
timeGNSS1 = VisionRTK.Gnss1.timestamp - VisionRTK.Gnss1.timestamp(1);
timeGNSS2 = VisionRTK.Gnss2.timestamp - VisionRTK.Gnss2.timestamp(1);

%Show the transformation from ECEF geodetic to ECEF cartesian
figure;
subplot(2,3,1)
plot(timeGNSS1, VisionRTK.Gnss1.longitude, '.', 'color', [0, 0.4470, 0.7410], 'markersize', 10)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.longitude, '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 10)
grid on;
ylabel('Longitude [°]')
set(gca, 'FontSize', 20);

subplot(2,3,2)
plot(timeGNSS1, VisionRTK.Gnss1.latitude, '.', 'color', [0, 0.4470, 0.7410], 'markersize', 10)
hold on;
plot(timeGNSS2, VisionRTK.Gnss2.latitude, '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 10)
grid on;
ylabel('Latitude [°]');
title('ECEF geodetic C.S','FontSize', 24)
set(gca, 'FontSize', 20);

subplot(2,3,3)
plot(timeGNSS1, VisionRTK.Gnss1.altitude, '.', 'color', [0, 0.4470, 0.7410], 'markersize', 12)
hold on;
plot(timeGNSS2, VisionRTK.Gnss2.altitude, '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 12)
grid on;
ylabel('Altitude [m]')
set(gca, 'FontSize', 20);

%Plot GNSS receivers on the ECEF Cartesian coordinate system
subplot(2,3,4)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.cartesian(:,1), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 12)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.cartesian(:,1), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 12)
grid on;
xlabel('Time [s]')
ylabel('X_{ECEF} [m]')
ytickformat('% .0f')
set(gca, 'FontSize', 20);

subplot(2,3,5)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.cartesian(:,2), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 12)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.cartesian(:,2), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 12)
grid on;
xlabel('Time [s]')
ylabel('Y_{ECEF} [m]');
ytickformat('% .0f')
title('ECEF cartesian C.S','FontSize', 24)
set(gca, 'FontSize', 20);


subplot(2,3,6)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.cartesian(:,3), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 12)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.cartesian(:,3), '.', 'color', [0.3010, 0.7450, 0.9330],  'markersize', 12)
grid on;
xlabel('Time [s]')
ylabel('Z_{ECEF} [m]')
ytickformat('% .0f')
set(gca, 'FontSize', 20);

%Show the transformation from ECEF cartesian to ENU
figure
subplot(2,3,1)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.cartesian(:,1), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 10)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.cartesian(:,1), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 10)
grid on;
ylabel('X_{ECEF} [m]')
set(gca, 'FontSize', 20);

subplot(2,3,2)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.cartesian(:,2), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 10)
hold on;
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.cartesian(:,2), '.', 'color', [0.3010, 0.7450, 0.9330],  'markersize', 10)
grid on;
ylabel('Y_{ECEF} [m]')
set(gca, 'FontSize', 20);

subplot(2,3,3)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.cartesian(:,3), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 10)
hold on;
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.cartesian(:,3), '.', 'color', [0.3010, 0.7450, 0.9330],  'markersize', 10)
grid on;
ylabel('Z_{ECEF} [m]')
set(gca, 'FontSize', 20);

%Plot the GNSS receivers in the ENU coordinate system
subplot(2,3,4)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.ENU(:,1), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 10)
hold on;
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.ENU(:,1), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 10)
grid on;
xlabel('Time [s]')
ylabel('East [m]')
set(gca, 'FontSize', 20);

subplot(2,3,5)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.ENU(:,2), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 10)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.ENU(:,2), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 10)
grid on;
xlabel('Time [s]')
ylabel('North [m]')
set(gca, 'FontSize', 20);

subplot(2,3,6)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.ENU(:,3), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 10)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.ENU(:,3), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 10)
grid on;
xlabel('Time [s]')
ylabel('Up [m]')
set(gca, 'FontSize', 20);

%Show the transformation from ENU to Local coordinate system
figure
subplot(2,3,1)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.ENU(:,1), '.', 'color', [0, 0.4470, 0.7410],  'markersize', 12)
hold on;
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.ENU(:,1), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 12)
grid on;
ylabel('East [m]')
set(gca, 'FontSize', 20);

subplot(2,3,2)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.ENU(:,2), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 12)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.ENU(:,2), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 12)
grid on;
title('ENU C.S','FontSize', 24)
ylabel('North [m]')
set(gca, 'FontSize', 20);

subplot(2,3,3)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.ENU(:,3), '.', 'color', [0, 0.4470, 0.7410],  'markersize', 12)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.ENU(:,3), '.', 'color', [0.3010, 0.7450, 0.9330],  'markersize', 12)
grid on;
ylabel('Up [m]')
set(gca, 'FontSize', 20);

%Plot the GNSS receivers in the local coordinate system
subplot(2,3,4)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.local(:,1), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 12)
hold on;
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.local(:,1), '.', 'color', [0.3010, 0.7450, 0.9330],  'markersize', 12)
grid on;
xlabel('Time [s]')
ylabel('X [m]')
set(gca, 'FontSize', 20);

subplot(2,3,5)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.local(:,2), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 12)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.local(:,2), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 12)
grid on;
xlabel('Time [s]')
ylabel('Y [m]')
title('Local C.S','FontSize', 24)
set(gca, 'FontSize', 20);

subplot(2,3,6)
plot(timeGNSS1, VisionRTK.Gnss1.trajectory.local(:,3), '.', 'color', [0, 0.4470, 0.7410], 'markersize', 12)
hold on; 
plot(timeGNSS2, VisionRTK.Gnss2.trajectory.local(:,3), '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 12)
grid on;
xlabel('Time [s]')
ylabel('Z [m]')
set(gca, 'FontSize', 20);

end 