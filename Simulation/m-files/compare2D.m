% Compare visually the trajectory in the xy plane depicted by the Fixposition solution and the sensorfusion  
%   compare2D.m 
%
%   See also PLOT, matlab built-in function
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%
function compare2D(Kuka, Mechanisation, Fixposition)
% inputs (3) : - Kuka
%              - Mechanisation
%              - Fixposition  
%   

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');
grid on;
% Create plot of trajectory based on Fixposition solution
plot(Fixposition.Data(:,1), Fixposition.Data(:,2),'DisplayName','Fixposition','MarkerSize',14,'Marker','.',...
    'LineStyle','none',...
    'Color',[1 0 0]);

% Create plot of the trajectory based on sensor fusion 
plot(Mechanisation.Data(:,1), Mechanisation.Data(:,2),'DisplayName','Mechanisation','MarkerSize',14,'Marker','.',...
    'LineStyle','none',...
    'Color',[0.96078431372549 0.815686274509804 0.231372549019608]);

% Create plot
plot(Kuka.Data(:,1),Kuka.Data(:,2),'DisplayName','Kuka','MarkerSize',12,'Marker','.',...
    'LineStyle','none',...
    'Color',[0 0 0]);
%Activate the GrId
grid on

% Create ylabel
ylabel('Y [m]');

% Create xlabel
xlabel('X [m] ');

% Create title
title('Trajectory of the platform in 2D');

box(axes1,'on');
axis(axes1,'padded');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'DataAspectRatio',[1 1 1],'FontSize',18,'PlotBoxAspectRatio',...
    [1 1.0009896640715 3.90889719123338]);
% Create legend
legend1 = legend(axes1,'show');
set(legend1,'Location','southeastoutside');

end