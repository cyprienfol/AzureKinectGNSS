% Visualise the hit of the laser beam on a target place on a planar surface.  
%   visualiseTarget.m 
%
%   See also PLOT matlab built-in function 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function visualiseTarget(intersectionKuka, intersectionFuse, Target)
% inputs (3) - intersectionFuse is a vector of double that represents all
%              intersections between the laser beam simulated and a imaginery vertical plane at a certain distance
%              based on the Fixposition navigationnal information 
%
%            - intersectionKuka is a vector of double that represents the
%              intersections between the laser beam simulated
%              and a imaginery vertical plane at a certain distance based
%              on the true navigationnal information (Kuka robot)
%
%            - Target is a 3x1 vector of double that represent the position of the 
%              center of the target aiming by the simulated laser beam
%

%% PART I: Compute the residuals 
%Statistical analysis of the results (Mean, Standard Deviation, Residuals)    
target_res = sqrt(sum((intersectionFuse' - Target.position').^2,2));
target_mean = mean(target_res);
target_std = std(target_res);

fprintf('2D-trajectory : \n');
fprintf('Position:\t mean: %f [m] \t standard deviation: %f [m]\n', target_mean ,target_std);

%Plot the ring of accuracy
th = 0:pi/50:2*pi;
% x_1cm = 0.01 * cos(th) + Target.position(2);
% y_1cm = 0.01 * sin(th) + Target.position(3);
% x_5cm = 0.05 * cos(th) + Target.position(2);
% y_5cm = 0.05 * sin(th) + Target.position(3);
% x_10cm = 0.1 * cos(th) + Target.position(2);
% y_10cm = 0.1 * sin(th) + Target.position(3);
% x_15cm = 0.15 * cos(th) + Target.position(2);
% y_15cm = 0.15 * sin(th) + Target.position(3);

% figure
% plot(Target.position(2), Target.position(3), 'p', 'markersize', 20);
% hold on
% plot(x_1cm, y_1cm, '-.k', 'linewidth', 1)
% plot(x_5cm, y_5cm, 'k', 'linewidth', 1.2)
% plot(x_10cm, y_10cm, '--k', 'linewidth', 1.4)
% plot(x_15cm, y_15cm, 'k', 'linewidth', 1.5)
% 
% plot(intersectionKuka(2,:),intersectionKuka(3,:), '.g' )
% plot(intersectionFuse(2,:),intersectionFuse(3,:), 'xr' )
% axis equal
% xlabel('y axis [m]')
% ylabel('z axis [m]')
% legend('Target', '1cm', '5cm', '10cm', '15cm','kuka', 'Fixposition', 'location', 'southeastoutside')
% title('Projection error analysis')

x_1cm = 0.05 * cos(th) + Target.position(2);
y_1cm = 0.05 * sin(th) + Target.position(3);
x_5cm = 0.10 * cos(th) + Target.position(2);
y_5cm = 0.10 * sin(th) + Target.position(3);
x_1dm = 0.15 * cos(th) + Target.position(2);
y_1dm = 0.15 * sin(th) + Target.position(3);
x_5dm = 0.20 * cos(th) + Target.position(2);
y_5dm = 0.20 * sin(th) + Target.position(3);
x_1m = 0.25 * cos(th) + Target.position(2);
y_1m = 0.25 * sin(th) + Target.position(3);


figure
plot(Target.position(2), Target.position(3), 'p', 'markersize', 20);
hold on;
grid on;
plot(x_1cm, y_1cm, 'k', 'linewidth', 1.8)
plot(x_5cm, y_5cm, '--k', 'linewidth', 1.6)
plot(x_1dm, y_1dm, 'k', 'linewidth', 1.4)
plot(x_5dm, y_5dm, '--k', 'linewidth', 1.2)
plot(x_1m, y_1m, 'k', 'linewidth', 1)

% plot(intersectionKuka(2,:),intersectionKuka(3,:), '.g' )
plot(intersectionFuse(2,:),intersectionFuse(3,:), 'xr' )
axis equal
xlabel('Y [m]')
ylabel('Z[m]')
legend('Target', '5cm', '10cm', '15cm', '20cm', '25cm', 'Laser hit', 'location', 'southeastoutside')
title('Target at 5 [m]')
axis padded
set(gca, 'FontSize', 25)





end