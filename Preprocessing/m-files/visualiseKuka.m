% Visualise the transformation applied to GNSS measurements
%   visualiseKuka.m 
%
%   See also PLOT, matlab built-in function, 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function visualiseKuka(Kuka)
% input(1) : - Kuka is a struct regrouping all the information related to
%              the Kuka robotic arm measurement. (ref. Ch.4 of report)
%

%Convert UNIX time into human reabible time 
timeKuka = Kuka.timestamp-Kuka.timestamp(1);

%Plot the measurements
figure
subplot(2,3,1)
plot(timeKuka(1:500), Kuka.trajectory.position(1:500,1), '.k');
grid on
ylim([-0.01, 0.01])
ylabel('X [m]', 'FontWeight', 'bold')
set(gca, 'Fontsize', 20)

subplot(2,3,2)
plot(timeKuka(1:500), Kuka.trajectory.position(1:500,2), '.k');
grid on
ylim([-0.01, 0.01])
ylabel('Y [m]', 'FontWeight', 'bold')
set(gca, 'Fontsize', 20)

subplot(2,3,3)
plot(timeKuka(1:500), Kuka.trajectory.position(1:500,3), '.k');
grid on
ylim([-0.01, 0.01])
ylabel('Z [m]', 'FontWeight', 'bold')
set(gca, 'Fontsize', 20)

subplot(2,3,4)
plot(timeKuka(1:500), (180/pi)*Kuka.trajectory.orientation(1:500,1), '.k');
grid on
ylim([-1, 1])
xlabel('Time [s]', 'FontWeight', 'bold')
ylabel('Roll [°]', 'FontWeight', 'bold')
set(gca, 'Fontsize', 20)

subplot(2,3,5)
plot(timeKuka(1:500), (180/pi)*Kuka.trajectory.orientation(1:500,2), '.k');
grid on
ylim([-1, 1])
xlabel('Time [s]', 'FontWeight', 'bold')
ylabel('Pitch [°]', 'FontWeight', 'bold')
set(gca, 'Fontsize', 20)

subplot(2,3,6)
plot(timeKuka(1:500), (180/pi)*Kuka.trajectory.orientation(1:500,3), '.k');
grid on
ylim([-1, 1])
xlabel('Time [s]', 'FontWeight', 'bold')
ylabel('Yaw [°]', 'FontWeight', 'bold')
set(gca, 'Fontsize', 20)

%Display the offset for each component
fprintf('Position offset : X = %f[m] \t Y = %f[m] \t Z = %f[m] \n ', mean(Kuka.trajectory.position(1:500,:)));
fprintf('Orientation offset : Roll = %f[°] \t Pitch = %f[°] \t Yaw = %f[°] \n', (180/pi)*mean(Kuka.trajectory.orientation(1:500,:)));


end