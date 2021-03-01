% Best Kalman filter implementation used as default 
% x = [Position, Velocity, Acceleration, Attitude, Angular Rate, Bias Acc., Bias Gyro.]'
% z = [Position GNSS, Heading Baseline, Acceleration Imu, Angular Rate Imu]';
%   KalmanFilter.m 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [xh, y, P, K] = KalmanFilter(z, xh_, P_, dt)
% inputs  (5)  : - z is a 10x1 double vector that represents the measurement vector from the
%                  Kalman Filter algorithm  
%
%                - xh_ is a 21x1 double vector that represents the state vector before filtering  
%
%                - P_ is a 21x21 double matrix that represents the covariance matrix of the
%                  state vector from the k-1 iteration  
%
%                - dt is a double that represent the difference between timesteps 
%
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
 
%% System Model
%State transition Matrix
F = [eye(3),  eye(3)*dt, eye(3)*0.5*(dt^2), zeros(3), zeros(3), zeros(3), zeros(3);...
     zeros(3),  eye(3), eye(3)*dt, zeros(3), zeros(3), zeros(3) zeros(3);...     
     zeros(3),  zeros(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3);... 
     zeros(3),  zeros(3), zeros(3), eye(3), eye(3)*dt, zeros(3), zeros(3);... 
     zeros(3),  zeros(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3);...
     zeros(3), zeros(3),  zeros(3), zeros(3), zeros(3), eye(3), zeros(3);...
     zeros(3), zeros(3), zeros(3),  zeros(3), zeros(3), zeros(3), eye(3)];

%System covariance matrix    
Q_pos_xy = [0.01^2*eye(2),zeros(2,1)];
Q_pos = [Q_pos_xy;0.1^2*[0,0,1]];
Q_vel = 0.1^2*eye(3);
Q_acc = 0.005^2*eye(3);
Q_att = 0.001^2*eye(3);
Q_ang = 0.0005^2*eye(3);
Q_bac = 0.05^2*eye(3);
Q_bgy = 0.05^2*eye(3);


%Assumption: no correlation between states
Q = [Q_pos, zeros(3), zeros(3), zeros(3), zeros(3),zeros(3), zeros(3);...
     zeros(3), Q_vel, zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);...
     zeros(3), zeros(3), Q_acc, zeros(3), zeros(3), zeros(3), zeros(3);...
     zeros(3), zeros(3), zeros(3), Q_att, zeros(3), zeros(3), zeros(3);...
     zeros(3), zeros(3), zeros(3), zeros(3), Q_ang, zeros(3), zeros(3);...
     zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), Q_bac, zeros(3);...
     zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), Q_bgy];
 
%% Measurement Model
%Measurement transition matrix 
H = [eye(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);...
     zeros(1,3), zeros(1,3), zeros(1,3), [0, 0, 1], zeros(1,3), zeros(1,3), zeros(1,3);...
     zeros(3), zeros(3), eye(3), zeros(3), zeros(3), eye(3), zeros(3);...
     zeros(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3), eye(3)];

%Measurement covariance matrix
%Measurement noise in the Gnss receivers
R_gnss = [0.005^2, 0, 0, 0;...
          0, 0.005^2, 0, 0;...
          0, 0, 0.003^2, 0
          0, 0, 0, 0.5*(pi/180)^2];
%Measurement noise in the imu
% sigma_acc = 0.04;
% sigma_gyro = 0.0013;
% sigma_acc = 1;
% sigma_gyro = 5;
sigma_accx = 0.04; sigma_accy = 0.04; sigma_accz = 0.1;  
R_acc = [sigma_accx^2, 0, 0;
         0, sigma_accy^2, 0;
         0, 0, sigma_accz^2;];

sigma_gyrx = 1000; sigma_gyry = 1000; sigma_gyrz = 0.001;     
R_gyro = [sigma_gyrx^2, 0, 0;
         0, sigma_gyry^2, 0;
         0, 0, sigma_gyrz^2;];

R_imu = [R_acc, zeros(3);
         zeros(3), R_gyro];
 
R = [R_gnss, zeros(length(R_gnss),length(R_imu));
     zeros(length(R_imu),length(R_gnss)), R_imu];

%% Algorithm
%Extrapolation Phase
xh_pr = F*xh_;
P_pr = F*P_*F'+ Q;

%Kalman Gain
K = (P_pr*H')/(H*P_pr*H' + R);

%Update Phase
xh = xh_pr + K*(z - H*xh_pr);
P = (eye(size(K*H))-K*H)*P_;

%actual deviation 
y = z - H*xh;

end
