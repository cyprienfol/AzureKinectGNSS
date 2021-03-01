% Perform the mechanisation of the IMU and build TRAJECTORY of the platform 
%   mechaniseIMU.m 
%
%   See also CUMSUM, matlab built-in function
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function trajectory = mechaniseIMU(Imu, Parameter)
% input (2)  : - Imu is a struct that regroup all the information from
%                IMU sensor 
%
%              - Parameter is a struct that contains the information influencing
%                the sensor fusion based on the experiment performed
%
% outputs (1): - trajectory is a struct that regroups all the navigationnal
%                information of the IMU after integration (position and
%                velocity)
%

%% Step 0: Estimate the parameter of intertial system
gravity= [0,0,9.81];
omega_e = (7.292115e-5)*[cosd(Parameter.center.longitude),0,-sind(Parameter.center.longitude)];  

%% STEP I: integrate the gyroscope readings
%Calculate the oreintation of the paltform
dt = diff(Imu.timestamp);
trajectory.angularrate = Imu.angularrate(1:end-1,:).*dt - omega_e;
trajectory.attitude = cumsum(trajectory.angularrate );
trajectory.attitude = [0,0,0; trajectory.attitude];

%% STEP II: Correct accelerometer readings
Rx = @(Phi)[ 1 0 0; 0 cos(Phi), sin(Phi); 0, -sin(Phi), cos(Phi)];
Ry = @(Theta)[cos(Theta), 0, -sin(Theta); 0, 1, 0; sin(Theta), 0, cos(Theta)];
Rz = @(Psi)[cos(Psi), sin(Psi), 0; -sin(Psi), cos(Psi), 0; 0, 0, 1];

%Project the accelerometer readings
for indAcc = 1:length(trajectory.attitude)
    %orient the accelerometer vector for each steps
    projectAccel(indAcc,:) = Rx(trajectory.attitude(indAcc,1))*Ry(trajectory.attitude(indAcc,2))*Rz(trajectory.attitude(indAcc,3))*Imu.specificforce(indAcc,:)';
end

trajectory.acceleration = projectAccel + gravity;

%% Step III: Integrate the acceleration 
%Create the position out of the IMU's raw data
trajectory.velocity = cumsum(trajectory.acceleration(1:end-1,:).*dt);
trajectory.position = cumsum(trajectory.velocity(1:end-1,:).*dt(2:end) + (1/2)*trajectory.acceleration(2:end-1,:).*(dt(2:end).^2));
% 
% %DEBUGGING
% figure; 
% subplot(2,3,1)
% plot(Imu.timestamp, trajectory.position(:,1), '.');
% subplot(2,3,2); 
% plot(Imu.timestamp, trajectory.position(:,2), '.');
% subplot(2,3,3); 
% plot(Imu.timestamp, trajectory.position(:,3), '.');
% subplot(2,3,4)
% plot(Imu.timestamp, trajectory.attitude(:,1), '.');
% subplot(2,3,5); 
% plot(Imu.timestamp, trajectory.attitude(:,2), '.');
% subplot(2,3,6); 
% plot(Imu.timestamp, trajectory.attitude(:,3), '.');

end