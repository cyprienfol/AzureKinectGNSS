% Compare the 2 GNSS receivers on board of the platform with Kuka Robot 
%   compareGNSS.m 
%
%   See also DATETIME, PLOT, matlab built-in function, 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function compareGNSS(Kuka, VisionRTK)
% inputs (2) : - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - VisionRTK is a struct regrouping all the information related to
%                the vision-RTK measurement. (ref. Ch.4 of report)

%Convert time to [HH:MM:SS] format
timeKuka = datetime(Kuka.timestamp, 'ConvertFrom', 'posixtime' );
timeGNSS1 = datetime(VisionRTK.Gnss1.timestamp,'ConvertFrom', 'posixtime');
timeGNSS2 = datetime(VisionRTK.Gnss2.timestamp,'ConvertFrom', 'posixtime');

%latitude GNSS comparison
figure
ax1 = subplot(3,1,1);
plot(timeKuka, Kuka.trajectory.position(:,2),'.k')
grid on
ylabel('Y [m]');
title('Kuka Robot');
set(gca, 'FontSize', 20);

ax2 = subplot(3,1,2);
plot(timeGNSS1 , VisionRTK.Gnss1.latitude, '.', 'color', [0, 0.4470, 0.7410], 'markersize', 10)
grid on
ylabel('Latitude [°]')
title('Left GNSS Receiver')
set(gca, 'FontSize', 20);

ax3 = subplot(3,1,3);
plot(timeGNSS2, VisionRTK.Gnss2.latitude, '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 10)
grid on
ylabel('Latitude [°]')
title('Right GNSS Receiver')
set(gca, 'FontSize', 20);

linkaxes([ax1,ax2, ax3],'x');
xlabel('Time [HH:MM:SS]')

%longitude GNSS comparison
figure
ax1 = subplot(3,1,1);
plot(timeKuka, Kuka.trajectory.position(:,2),'.k', 'markersize', 12)
grid on
ylabel('Y [m]');
set(gca, 'FontSize', 25);
title('Kuka Robot');

ax2 = subplot(3,1,2);
plot(timeGNSS1, VisionRTK.Gnss1.longitude, '.', 'color', [0, 0.4470, 0.7410], 'markersize', 12)
grid on
ylabel('Longitude [°]')
yticks([8.510354, 8.510360, 8.510366])
set(gca, 'FontSize', 25);
title('Left GNSS Receiver');

ax3 = subplot(3,1,3);
plot(timeGNSS2, VisionRTK.Gnss2.longitude, '.', 'color', [0.3010, 0.7450, 0.9330], 'markersize', 12)
grid on
axis padded
ylabel('Longitude [°]')
xlabel('Time [HH:MM:SS]')
yticks([8.510356, 8.510362, 8.510368])
set(gca, 'FontSize', 25);
title('Right GNSS Receiver')
linkaxes([ax1,ax2, ax3],'x');
axis 'auto x';

end