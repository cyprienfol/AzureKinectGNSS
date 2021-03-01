% Compare each components of the sensor fusion position and orientation with Ground truth and Fixposition solution  
%   compare3D.m 
%
%   See also PLOT, matlab built-in function
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function compare3D(Kuka, Mechanisation, Fixposition)
% inputs (3) : - Kuka
%              - Mechanisation
%              - Fixposition  
%   

%Position
figure;
subplot(3,1,1)
plot(Kuka.Time, Kuka.Data(:,1), '.');
hold on;
plot(Mechanisation.Time, Mechanisation.Data(:,1), '.');
plot(Fixposition.Time, Fixposition.Data(:,1), '.');
legend('Kuka', 'Mechanisation', 'Fixposition', 'location', 'southeastoutside')
xlabel('Time [sec] ');
ylabel('X local [m]');
title('x_{platform}');
ax = gca;
ax.FontSize = 15;

subplot(3,1,2)
plot(Kuka.Time, Kuka.Data(:,2), '.');
hold on;
plot(Mechanisation.Time, Mechanisation.Data(:,2), '.');
plot(Fixposition.Time, Fixposition.Data(:,2), '.');
legend('Kuka', 'Mechanisation', 'Fixposition', 'location', 'southeastoutside')
xlabel('Time [sec] ');
ylabel('Y local [m]');
title('y_{platform}');
ax = gca;
ax.FontSize = 15;

subplot(3,1,3)
plot(Kuka.Time, Kuka.Data(:,3), '.');
hold on;
plot(Mechanisation.Time, Mechanisation.Data(:,3), '.');
plot(Fixposition.Time, Fixposition.Data(:,3), '.');
legend('Kuka', 'Mechanisation', 'Fixposition', 'location', 'southeastoutside')
xlabel('Time [sec] ');
ylabel('Z local [m]');
title('z_{platform}');
ax = gca;
ax.FontSize = 15;

sgtitle('Position')

%attitude
figure;
subplot(3,1,1)
plot(Kuka.Time, Kuka.Data(:,4), '.');
hold on;
plot(Mechanisation.Time, Mechanisation.Data(:,4), '.');
plot(Fixposition.Time, Fixposition.Data(:,4), '.');
legend('Kuka', 'Mechanisation', 'Fixposition', 'location', 'southeastoutside')
xlabel('Time [sec] ');
ylabel('\Sigma_{x} local [rad]');
title('roll');
ax = gca;
ax.FontSize = 15;

subplot(3,1,2)
plot(Kuka.Time, Kuka.Data(:,5), '.');
hold on;
plot(Mechanisation.Time, Mechanisation.Data(:,5), '.');
plot(Fixposition.Time, Fixposition.Data(:,5), '.');
legend('Kuka', 'Mechanisation', 'Fixposition', 'location', 'southeastoutside')
xlabel('Time [sec] ');
ylabel('\Sigma_{y} local [rad]');
title('pitch ');
ax = gca;
ax.FontSize = 15;

subplot(3,1,3)
plot(Kuka.Time, Kuka.Data(:,6), '.');
hold on;
plot(Mechanisation.Time, Mechanisation.Data(:,6), '.');
plot(Fixposition.Time, Fixposition.Data(:,6), '.');
legend('Kuka', 'Mechanisation', 'Fixposition', 'location', 'southeastoutside')
xlabel('Time [sec] ');
ylabel('\Sigma_{z} local [rad]');
title('yaw');
ax = gca;
ax.FontSize = 15;

sgtitle('attitude')

end