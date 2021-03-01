% First Kalman Filter architecture variation:
% x = [Position, Velocity, Attitude, Bias acc., Bias gyro. ]'
% u = [Imu acceleration, Imu angular rate]';
% z = [Position GNSS, Heading Baseline];
%   KF1.m 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [xh, y, P, K] = KF1(z, xh_, u, P_, dt)
% inputs  (5)  : - z is a 10x1 double vector that represents the measurement vector from the
%                  Kalman Filter algorithm  
%
%                - xh_ is a 21x1 double vector that represents the state vector before filtering  
%
%                - u is a 6x1 double vector that represents the input vector from the kalman filter
%
%                - P_ is a 21x21 double matrix that represents the covariance matrix of the
%                  state vector from the k-1 iteration  
%
%                - dt is a double that represent the difference between timesteps  
%
% outputs (4)  : - xh is a 21x1 double vector that represents the state vector after filtering  
%
%                - y is a 10x1 double vector that represents the innovation
%                  vector from the Kalman Filter between iterations
%
%                - P is a 21x21 double matrix that represents the covariance matrix of the
%                  state vector from the k iteration
%
%                - K is a 10x10 double matrix that represents the kalman
%                  gain of the filter in between iteration k-1 and k
%
%

%% System Model
%State transition Matrix
F = [eye(3),  eye(3)*dt, zeros(3), eye(3)*0.5*(dt^2), zeros(3);...
     zeros(3),  eye(3), zeros(3), eye(3)*dt, zeros(3);...     
     zeros(3),  zeros(3), eye(3), zeros(3), eye(3)*dt;... 
     zeros(3),  zeros(3), zeros(3), eye(3), zeros(3);... 
     zeros(3),  zeros(3), zeros(3), zeros(3), eye(3)];

%Input transition Matrix
G = [eye(3)*0.5*(dt^2), zeros(3);...
     eye(3)*dt, zeros(3);...
     zeros(3), eye(3)*dt;... 
     zeros(3), zeros(3);...
     zeros(3), zeros(3)];

%System covariance matrix    
Q_pos = 0.01^2*eye(3);
Q_vel = 0.1^2*eye(3);
Q_att = 0.01^2*eye(3);
Q_bac = 0.005^2*eye(3);
Q_bgy = 0.1^2*eye(3);

%Assumption: no correlation between states
Q = [Q_pos, zeros(3), zeros(3), zeros(3), zeros(3);...
     zeros(3), Q_vel, zeros(3), zeros(3), zeros(3);...
     zeros(3), zeros(3), Q_att, zeros(3), zeros(3);...
     zeros(3), zeros(3), zeros(3), Q_bac, zeros(3);...
     zeros(3), zeros(3), zeros(3), zeros(3), Q_bgy];
 
%% Measurement Model
%Measurement transition matrix 
H = [eye(3), zeros(3), zeros(3), zeros(3), zeros(3);
     zeros(1,3), zeros(1,3), [0, 0, 1], zeros(1,3), zeros(1,3)];

%Measurement covariance matrix     
R = [0.005^2, 0, 0, 0;...
     0, 0.005^2, 0, 0;...
     0, 0, 0.001^2, 0
     0, 0, 0, (pi/180)^2];

%% Algorithm
%Extrapolation Phase
xh_pr = F*xh_ + G*u;
P_pr = F*P_*F'+ Q;

%Kalman Gain
K = (P_pr*H')/(H*P_pr*H' + R);

%Update Phase
xh = xh_pr + K*(z - H*xh_pr);
P = (eye(size(K*H))-K*H)*P_;

%actual deviation 
y = z - H*xh;

end
