% Apply a Loosely coupled integration to IMU measurements with aids of GNSS receivers measurements
%   fuseIMUGNSS.m 
%
%   See also KF1, KF2, KF3 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function Fusion = fuseIMUGNSS(f, omega, x_local1, x_local2, b_acc, b_gyro, timeImu, timeGnss, Parameter , flagkf)
% input (1)  :  - Fusion is a struct regrouping all the information related to
%                 the sensor Fusion. (ref. Ch.4 of report)
%
% outputs (10): - f is nx3 double vector that represent the specific force
%                 from the Azure Kinect IMU
%
%               - omega is nx3 double vector that represent the angular
%                 rate from the Azure Kinect IMU
%
%               - b_acc is nx3 double vector that represent the biases
%                 of the accelerometer from the Azure Kinect IMU
%
%               - b_gyro is nx3 double vector that represent the biases
%                 of the gyroscope from the Azure Kinect IMU
%
%               - x_local1 is nx3 double vector that represent the
%                 position of the left GNSS receiver in the local
%                 coordinate system
%
%               - x_local2 is nx3 double vector that represent the
%                 position of the right GNSS receiver in the local
%                 coordinate system
%
%               - timeImu is the timesteps vectors of the timeseries
%                 from the GNSS receivers
%
%               - timeGNSS is the timesteps vectors of the timeseries
%                 from the GNSS receivers
%
%               - Parameter contains the information influencing
%                the sensor fusion based on the experiment performed
%
%               - flagkf is an integer include in {0,1,2}. The value of flagkf define which 
%                 Kalman filter variation is applied. By default flagkf = 0 


%Parameters
g = [0;0;9.81]; %gavity vector
omega_e = (7.292115e-5)*[cosd(Parameter.center.longitude),0,-sind(Parameter.center.longitude)]; 

%Initialize the rotation matrix
Rx = @(Phi)[ 1 0 0; 0 cos(Phi), sin(Phi); 0, -sin(Phi), cos(Phi)];
Ry = @(Theta)[cos(Theta), 0, -sin(Theta); 0, 1, 0; sin(Theta), 0, cos(Theta)];
Rz = @(Psi)[cos(Psi), sin(Psi), 0; -sin(Psi), cos(Psi), 0; 0, 0, 1];

%Get the time and iteration parameters
iterGnss = 1; 
dt = diff(timeImu);
orientation0 = 0;

%Create Baseline 
baseline = x_local1 - x_local2;
normebaseline = sqrt(dot(baseline,baseline,2));
initialheading = baseline(1,:);


%Start the iterative process
for iterImu = 1:length(dt)
    %Get the position in the ECEF cartesian reference frame
    z_gnss(1:3,iterGnss) = 0.5*(x_local1(iterGnss,:) + x_local2(iterGnss,:));    

    %Get the heading from the gnss baseline
    ratio = (baseline(iterGnss,:)*initialheading')/(norm(initialheading)*normebaseline(iterGnss,1));

    %Check the ratio is included in [-1,1]
    if ratio > 1
        ratio = 1;
    elseif ratio <-1
        ratio = -1;
    end

    %get heading from the scalar product formula
    heading = acos(ratio);

    %define sign of the rotation based on the cross product
    headingSign = cross(initialheading,baseline(iterGnss,:));
    z_gnss(4,iterGnss) = sign(headingSign(3))*heading;
    
    %% Step I: Compute the Accelaration based on the IMU data
    %Get the Orientation from gyroscope
    orientation(iterImu,:) = orientation0 + omega(iterImu,:)*dt(iterImu) + omega_e;
    
    switch flagkf
        case 1 %first version of the Kalman Filter
            %Project the specific force and correct gravity
            u_imu(1:3,iterImu) = Rx(orientation(iterImu,1))*Ry(orientation(iterImu,2))*Rz(orientation(iterImu,3))*(f(iterImu,:)') + g;  

            %Get the angular rate of the platform
            u_imu(4:6,iterImu) = omega(iterImu,:); 
            
            %Create the measurement vector
            z(:,iterImu) = z_gnss(:,iterGnss);
            
            %% Step II: Fuse the data with a Kalman Filter
            %initialisation of the filter for the first step
            if iterImu == 1    
                %Fill estimations for the initial state vector and the covariance matrix
                x0 = [z_gnss(1:3,1);zeros(3,1);zeros(3,1);b_acc*ones(3,1);b_gyro*ones(3,1)];
                P0 = [0.1*eye(3),  zeros(3), zeros(3), zeros(3), zeros(3);...
                      zeros(3),  0.1*eye(3), zeros(3), zeros(3), zeros(3);...     
                      zeros(3),  zeros(3), 0.01*eye(3), zeros(3), zeros(3);... 
                      zeros(3),  zeros(3), zeros(3), 0.01*eye(3), zeros(3);... 
                      zeros(3),  zeros(3), zeros(3), zeros(3), 0.01*eye(3)];

                %Run first iteration with estimations 
                %First version with input vector u 
                [state(:,iterImu), residuals(:,iterImu), cov_matrix(:,:,iterImu)] = KF1(z(:,iterImu), x0, u_imu(:,iterImu), P0, dt(iterImu));

            else 
                [state(:,iterImu), residuals(:,iterImu), cov_matrix(:,:,iterImu)] = KF1(z(:,iterImu), state(:,iterImu-1), u_imu(:,iterImu), cov_matrix(:,:,iterImu-1), dt(iterImu));
            
            end
            
        case 2 %Second version of the Kalman Filter
            %Project the specific force and correct gravity
            z_imu(1:3,iterImu) = Rx(orientation(iterImu,1))*Ry(orientation(iterImu,2))*Rz(orientation(iterImu,3))*(f(iterImu,:)') + g;  

            %Get the angular rate of the platform
            z_imu(4:6,iterImu) = omega(iterImu,:); 
            
            %Combine the measurement 
            z(:,iterImu) = [z_gnss(:,iterGnss); z_imu(:,iterImu)];
            
            %Create the input vector with bias
            u_bias = [b_acc*ones(3,1);b_gyro*ones(3,1)];

            %% Step II: Fuse the data with a Kalman Filter
            %initialisation of the filter for the first step
            if iterImu == 1    
                %Fill estimations for the initial state vector and the covariance matrix
                x0 = [z_gnss(1:3,1);zeros(3,1);zeros(3,1);zeros(3,1);zeros(3,1)];
                P0 = [0.1*eye(3),  zeros(3), zeros(3), zeros(3), zeros(3);...
                      zeros(3),  0.1*eye(3), zeros(3), zeros(3), zeros(3);...     
                      zeros(3),  zeros(3), 0.01*eye(3), zeros(3), zeros(3);... 
                      zeros(3),  zeros(3), zeros(3), 0.01*eye(3), zeros(3);... 
                      zeros(3),  zeros(3), zeros(3), zeros(3), 0.01*eye(3)];

                %Run first iteration with estimations 
                %First version with input vector u 
                [state(:,iterImu), residuals(:,iterImu), cov_matrix(:,:,iterImu)] = KF2(z(:,iterImu), x0, u_bias, P0, dt(iterImu));

            else 
                [state(:,iterImu), residuals(:,iterImu), cov_matrix(:,:,iterImu)] = KF2(z(:,iterImu), state(:,iterImu-1), u_bias, cov_matrix(:,:,iterImu-1), dt(iterImu));
            
            end
        
        otherwise %Default filter 
            %Project the specific force and correct gravity
            z_imu(1:3,iterImu) = Rx(orientation(iterImu,1))*Ry(orientation(iterImu,2))*Rz(orientation(iterImu,3))*(f(iterImu,:)') + g;  

            %Get the angular rate of the platform
            z_imu(4:6,iterImu) = omega(iterImu,:); 

            %Combine the measurement 
            z(:,iterImu) = [z_gnss(:,iterGnss); z_imu(:,iterImu)];

            %% Step II: Fuse the data with a Kalman Filter
            %initialisation of the filter for the first step
            if iterImu == 1    
                %Fill estimations for the initial state vector and the covariance matrix
                x0 = [z_gnss(1:3,1);zeros(3,1);zeros(3,1);zeros(3,1);zeros(3,1); b_acc*zeros(3,1); b_gyro*zeros(3,1)];
                P0 = [0.1*eye(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);...
                      zeros(3), 0.1*eye(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);...     
                      zeros(3), zeros(3), 0.01*eye(3), zeros(3), zeros(3), zeros(3), zeros(3);... 
                      zeros(3), zeros(3), zeros(3), 0.01*eye(3), zeros(3), zeros(3), zeros(3);... 
                      zeros(3), zeros(3), zeros(3), zeros(3), 0.01*eye(3), zeros(3), zeros(3);...
                      zeros(3), zeros(3),  zeros(3), zeros(3), zeros(3), 0.01*eye(3), zeros(3);...
                      zeros(3), zeros(3), zeros(3),  zeros(3), zeros(3), zeros(3), 0.01*eye(3)];

                %Run first iteration with estimations 
                %First version with input vector u 
                [state(:,iterImu), residuals(:,iterImu), cov_matrix(:,:,iterImu)] = KalmanFilter(z(:,iterImu), x0, P0, dt(iterImu));

            else 
                [state(:,iterImu), residuals(:,iterImu), cov_matrix(:,:,iterImu)] = KalmanFilter(z(:,iterImu), state(:,iterImu-1), cov_matrix(:,:,iterImu-1), dt(iterImu));

            end
    
    end

    %Update the Gnss measurement index
    if(iterGnss+1 <= length(timeGnss))
        if(timeGnss(iterGnss+1) < timeImu(iterImu+1)) 
             iterGnss = iterGnss+1;

        end

    else
        %warning("GNSS signal not received anymore");

    end

%     orientation0 = state(7:9,iterImu)';
    orientation0 = orientation(iterImu,:);

end

switch flagkf
    case 1
        %Save the timestamp of the fuse solution
        Fusion.timestamp(1,:) = timeImu(2:end); %first state is estimated

        %Exctract the value for visualization 
        Fusion.position = state(1:3,:);
        Fusion.velocity = state(4:6,:);
        Fusion.attitude = state(7:9,:);
        Fusion.biasAcc = state(10:12,:);
        Fusion.biasGyro = state(13:15,:);
        Fusion.measurement = z;
        Fusion.residuals = residuals;
        Fusion.cov_mat = cov_matrix;
    case 2
        %Save the timestamp of the fuse solution
        Fusion.timestamp(1,:) = timeImu(2:end); %first state is estimated

        %Exctract the value for visualization 
        Fusion.position = state(1:3,:);
        Fusion.velocity = state(4:6,:);
        Fusion.acceleration = state(7:9,:);
        Fusion.attitude = state(10:12,:);
        Fusion.angularrate = state(13:15,:);
        Fusion.measurement = z;
        Fusion.residuals = residuals;
        Fusion.cov_mat = cov_matrix;
   
    otherwise
        %Save the timestamp of the fuse solution
        Fusion.timestamp(1,:) = timeImu(2:end); %first state is estimated

        %Exctract the value for visualization 
        Fusion.position = state(1:3,:);
        Fusion.velocity = state(4:6,:);
        Fusion.acceleration = state(7:9,:);
        Fusion.attitude = state(10:12,:);
        Fusion.angularrate = state(13:15,:);
        Fusion.biasAcc = state(16:18,:);
        Fusion.biasGyro = state(19:21,:);
        Fusion.measurement = z;
        Fusion.residuals = residuals;
        Fusion.cov_mat = cov_matrix;
end
end
