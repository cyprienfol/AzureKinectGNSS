% Plot the confidence ellipse of the projection over the whole trajectory
%   compareProjection.m 
%
%   See also PLOT, matlab built-in function
%            CALCULATEELLIPSE
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function compareProjection(Hit, Target)
% inputs (2) : - Hit is a vector of double that represents the
%                intersections between the laser beam simulated
%                and a imaginery vertical plane at a certain distance
%
%              - Target is a 3x1 vector of double that represent the position of the 
%                center of the target aiming by the simulated laser beam
%   

%% PART I: Compute the residuals 
% %Statistical analysis of the results (Mean, Standard Deviation, Residuals)    

projection_err = (Hit(2:3,:) - Target.position(2:3))';
[ellipse_2sigma, X_fixp, Y_fixp]  = calculateEllipse(projection_err, 95);

%plot the ellips and the residuals

figure
% Plot the original data
plot(projection_err(:,1), projection_err(:,2), '.r', 'markersize', 14);
hold on;
mindata = min(min( projection_err));
maxdata = max(max( projection_err));
xlim([mindata-3, maxdata+3]);
ylim([mindata-3, maxdata+3]);

%Plot the target
plot(Target.position(2), Target.position(3), 'bp', 'markersize', 30);

% Draw the error ellipse
plot(ellipse_2sigma(:,1) + X_fixp, ellipse_2sigma(:,2) + Y_fixp,'--k', 'Linewidth', 4);
% Set the axis labels
hXLabel = xlabel('Y [m]');
hYLabel = ylabel('Z [m]');
grid on
legend('Laser hit', 'Target', 'Ellispse', 'location', 'southeastoutside');
set(gca, 'FontSize', 25)
title('95% confidence ellipse','FontSize', 30)


end