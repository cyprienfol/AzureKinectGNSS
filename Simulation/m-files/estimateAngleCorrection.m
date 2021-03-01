% Estimate the Euler angle correction to steer the laser towards the TARGET 
% for the inputs of the Kalman filter
%   exctratKFdata.m 
%
%   See also ATAN2, DOT, CROSS, SIGN, matlab built-in function  
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function eulerangle = estimateAngleCorrection(position, attitude, initialheading, target)
% inputs (4) : - position is 3x1 vector of double representing the position of the platform 
%
%              - orientation is 3x1 vector of double representing the orientation of the platform 
%
%              - initialheading is 3x1 vectors of double representing the
%                heading at the beginning of the trajectory    
%
%              - target is a 3x1 vector of double that represent the position of the 
%                center of the target aiming by the simulated laser beam
%
% output (1) : - eulerangle is a 3x1 vector representing the angle to rotate
%                the platform to steer in direction of the target
%

%Initialize the rotation matrix counter clockwise
Rx_ccw = @(Phi)[ 1 0 0; ... 
                 0 cos(Phi), -sin(Phi);...
                 0, sin(Phi), cos(Phi)];
             
Ry_ccw = @(Theta)[cos(Theta), 0, sin(Theta);...
                  0, 1, 0;...
                  -sin(Theta), 0, cos(Theta)];
              
Rz_ccw = @(Psi)[cos(Psi), -sin(Psi), 0;...
                sin(Psi), cos(Psi), 0;...
                0, 0, 1];

%Initialize the rotation matrix clockwise
Rx_cw = @(Phi)[ 1 0 0;...
                0 cos(Phi), sin(Phi);...
                0, -sin(Phi), cos(Phi)];
            
Ry_cw = @(Theta)[cos(Theta), 0, -sin(Theta);...
                 0, 1, 0; ...
                 sin(Theta), 0, cos(Theta)];

Rz_cw = @(Psi)[cos(Psi), sin(Psi), 0;...
               -sin(Psi), cos(Psi), 0;...
               0, 0, 1];

%% STEP 0: Prepare data
baseline = target - position;
currentheading = Rx_cw(attitude(1))*Ry_cw(attitude(2))*Rz_cw(attitude(3))*initialheading;

%% STEP I: Align target to the x axis 
%Find the first angle on the xy plane between heading and baseline
hxy = currentheading(1:2);
bxy = baseline(1:2); 

ratiogamma = dot(hxy,bxy)/(norm(hxy)*norm(bxy));

%Check ratio is included in [-1,1] to avoid complex results
if ratiogamma > 1
    ratiogamma = 1;
elseif ratiogamma <-1
    ratiogamma = -1;
end

%get the angle between the two vectors
gamma = acos(ratiogamma);

%Check with the cross product the sens of rotation
if sign(hxy(1)*bxy(2) - hxy(2)*bxy(1)) < 0
    Rz= Rz_cw(gamma);
    
else 
    Rz= Rz_ccw(gamma);

end

%% STEP II: Angle between the new target vector and the x axis
%Find the second angle on the x'z' plane between heading and baseline
%Perform a dot product between the two vectos in 3D
partialheading= Rz*currentheading; 
ratiobeta = dot(partialheading,baseline)/norm(baseline)*norm(partialheading);

%Check ratio is included in [-1,1] to avoid complex results
if ratiobeta > 1
    ratiobeta = 1;
elseif ratiobeta <-1
    ratiobeta = -1;
end

%get the angle between the two vectors
beta = acos(ratiobeta);

%Create the axis of rotation by taking the normal of the two vectors
axis_rot = cross(partialheading, baseline);
%Check that the vectors are not coplanar
if(norm(axis_rot) > 1e-6)    
    ux = axis_rot(1)/norm(axis_rot); uy = axis_rot(2)/norm(axis_rot); uz = axis_rot(3)/norm(axis_rot); 
    c = cos(beta); s = sin(beta);

    Ry = [(ux^2)*(1 - c) + c,     ux*uy*(1 - c) - uz*s,   ux*uz*(1 - c) + uy*s;...
           ux*uy*(1 - c) + uz*s, (uy^2)*(1 - c) + c,      uy*uz*(1 - c) - ux*s;...
           ux*uz*(1 - c) - uy*s,  uy*uz*(1 - c) + ux*s , (uz^2)*(1 - c) + c];

   Trans_mat = Ry*Rz;
else
    Trans_mat = Rz;
end

%Plot the rotation of the heading decompose in the 2 rotation 
% figure 
% plot3( [position(1), position(1) + initialheading(1)],  [position(2), position(2) + initialheading(2)],  [position(3), position(3) + initialheading(3)] )
% hold on
% plot3( [position(1), position(1) + currentheading(1)],  [position(2), position(2) + currentheading(2)],  [position(3), position(3) + currentheading(3)] )
% plot3( [position(1), position(1) + baseline(1)],  [position(2), position(2) + baseline(2)],  [position(3), position(3) + baseline(3)] )
% plot3(target(1), target(2), target(3), 'p')
% plot3( [position(1), position(1) + partialheading(1)],  [position(2), position(2) + partialheading(2)],  [position(3), position(3) + partialheading(3)] )
% finalheading = Ry*partialheading;
% plot3( [position(1), position(1) + finalheading(1)],  [position(2), position(2) + finalheading(2)],  [position(3), position(3) + finalheading(3)] )


%Clock-wise formula
eulerangle = zeros(3,1);
% eulerangle2 = zeros(3,1); %second solution

if(abs(Trans_mat(3,1)) ~= 1)
    eulerangle(2,1) = -asin(Trans_mat(1,3));
%     eulerangle2(2,1) = pi -eulerangle1(2,1); 
    
    eulerangle(1,1) = atan2(Trans_mat(2,3)/cos(eulerangle(2,1)),Trans_mat(3,3)/cos(eulerangle(2,1)));
%     eulerangle2(1,1) = atan2(Trans_mat(2,3)/cos(eulerangle2(2,1)),Trans_mat(3,3)/cos(eulerangle2(2,1)));
     
    eulerangle(3,1) = atan2(Trans_mat(1,2)/cos(eulerangle(2,1)),Trans_mat(1,1)/cos(eulerangle(2,1)));
%     eulerangle2(3,1) = atan2(Trans_mat(1,2)/cos(eulerangle2(2,1)),Trans_mat(1,1)/cos(eulerangle2(2,1)));

else
    eulerangle(2,1) = 0;
    
    if Trans_mat(3,1) == -1
        eulerangle(1,1) = pi/2;
        eulerangle(3,1) =  eulerangle(2,1) + atan2(Trans_mat(1,2), Trans_mat(1,3));
        
    else
        eulerangle(1,1) = -pi/2;
        eulerangle(3,1) =  -eulerangle(2,1) + atan2(-Trans_mat(1,2), -Trans_mat(1,3));
        
    end
         
end

end