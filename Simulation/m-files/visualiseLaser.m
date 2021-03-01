% Visualise the laser beam in 3D originateing from the platform to the target at each position of the trajectory.  
%   visualiseLaser.m 
%
%   See also PLOT3 matlab built-in function 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function visualiseLaser(positionKuka, attitudeKuka, correctionKuka, correctionFuse, initialheading, Target)
% inputs (6) : - positionKuka is a 3x1 vector representing the true position of
%                the platform based on Kuka measurements 
%
%              - attitudeKuka is a 3x1 vector representing the true orientation of
%                the platform based on Kuka measurements 
%
%              - correctionKuka is a 3x1 vector representing the angle to rotate
%                the platform to steer in direction of the target based on
%                Kuka navigationnal information
%
%              - correctionFuse is a 3x1 vector representing the angle to rotate
%                the platform to steer in direction of the target based on
%                Fixposition navigationnal information
%
%              - initialheading is 3x1 vectors of double representing the
%                heading at the beginning of the trajectory    
%
%              - Target is a 3x1 vector of double that represent the position of the 
%                center of the target aiming by the simulated laser beam
%

%Rotation Matrix initialisation
Rx = @(Phi)[ 1 0 0;...
                0 cos(Phi), sin(Phi);...
                0, -sin(Phi), cos(Phi)];
            
Ry = @(Theta)[cos(Theta), 0, -sin(Theta);...
                 0, 1, 0; ...
                 sin(Theta), 0, cos(Theta)];

Rz = @(Psi)[cos(Psi), sin(Psi), 0;...
               -sin(Psi), cos(Psi), 0;...
               0, 0, 1];
           
%create a new figure
figure
%Plot in 3D the position of the target target
plot3(Target.position(1), Target.position(2), Target.position(3), 'p', 'markersize', 30);
hold on 

%Plot the position and orientation of the platform
for ind_plot = 1:length(positionKuka)
    %plot in 3D the position given by kuka
    plot3(positionKuka(1,ind_plot), positionKuka(2, ind_plot), positionKuka(3, ind_plot),'*g', 'markersize', 20)  
    %plot in 3D orientation given by kuka 
    headingKuka = Rx(attitudeKuka(1,ind_plot))*Ry(attitudeKuka(2, ind_plot))*Rz(attitudeKuka(3, ind_plot) )*initialheading;
    laserKuka = Rx(correctionKuka(1,ind_plot))*Ry(correctionKuka(2, ind_plot))*Rz(correctionKuka(3, ind_plot) )*headingKuka;     
    
    plot3([positionKuka(1, ind_plot), positionKuka(1, ind_plot) + 10*laserKuka(1)], [positionKuka(2, ind_plot), positionKuka(2, ind_plot) + 10*laserKuka(2)],...
            [positionKuka(3, ind_plot), positionKuka(3, ind_plot) + 10*laserKuka(3)],'g', 'linewidth', 2);
   
    %plot in 3D orientation given by the Fusion
%     headingFuse = Rx(attitudeFuse(1,ind_plot))*Ry(attitudeFuse(2, ind_plot))*Rz(attitudeFuse(3, ind_plot) )*initialheading;
    laserFuse = Rx(correctionFuse(1,ind_plot))*Ry(correctionFuse(2, ind_plot))*Rz(correctionFuse(3, ind_plot) )*headingKuka;  
    
    plot3([positionKuka(1, ind_plot), positionKuka(1, ind_plot) + 10*laserFuse(1)], [positionKuka(2, ind_plot), positionKuka(2, ind_plot) + 10*laserFuse(2)],...
            [positionKuka(3, ind_plot), positionKuka(3, ind_plot) + 10*laserFuse(3)],'m', 'linewidth', 2);

end



xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
xlim([-0.5, 3.1])
grid on

%create a new figure
figure
%Plot in 3D the position of the target target
plot3(Target.position(1), Target.position(2), Target.position(3), 'p', 'markersize', 30);
hold on 

%Plot the position and orientation of the platform
plot3(positionKuka(1,:), positionKuka(2, :), positionKuka(3, :),'k', 'linewidth', 3);
plot3(positionKuka(1,200), positionKuka(2, 200), positionKuka(3, 200),'*g', 'markersize', 20)  
%plot in 3D orientation given by kuka 
headingKuka = Rx(attitudeKuka(1,200))*Ry(attitudeKuka(2,200))*Rz(attitudeKuka(3,200) )*initialheading;
laserKuka = Rx(correctionKuka(1,200))*Ry(correctionKuka(2,200))*Rz(correctionKuka(3,200) )*headingKuka;     

plot3([positionKuka(1,200), positionKuka(1,200) + 10*laserKuka(1)], [positionKuka(2,200), positionKuka(2,200) + 10*laserKuka(2)],...
        [positionKuka(3,200), positionKuka(3,200)+ 10*laserKuka(3)],'g', 'linewidth', 2);

%plot in 3D orientation given by the Fusion
%     headingFuse = Rx(attitudeFuse(1,ind_plot))*Ry(attitudeFuse(2, ind_plot))*Rz(attitudeFuse(3, ind_plot) )*initialheading;
laserFuse = Rx(correctionFuse(1,200))*Ry(correctionFuse(2, 200))*Rz(correctionFuse(3, 200) )*headingKuka;  

plot3([positionKuka(1, 200), positionKuka(1, 200) + 10*laserFuse(1)], [positionKuka(2, 200), positionKuka(2, 200) + 10*laserFuse(2)],...
        [positionKuka(3, 200), positionKuka(3, 200) + 10*laserFuse(3)],'m', 'linewidth', 2);


xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
xlim([-0.5, 3.1]);
ylim([-0.5, 0.5]);
zlim([-0.1,1]);
grid on

end