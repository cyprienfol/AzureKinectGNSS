% Visualise the efficiency of the sensor Fusion comparing it with ground truth on plots  
%   visualiseFusion.m 
%
%   See also CONVERTLOCALTOENU, CONVERTENUTOCARTESIAN, 
%            VISUALISE2DMOTION, VISUALISE3DMOTION, COMPAREFUSION 
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function visualiseFusion(Fusion, Kuka, Parameter)
% inputs (3) : - Fusion is a struct regrouping all the information related to
%                the sensor Fusion. (ref. Ch.4 of report)
%
%              - Kuka is a struct regrouping all the information related to
%                the Kuka robotic arm measurement. (ref. Ch.4 of report)
%
%              - Parameter is a struct that contains the information influencing
%                the sensor fusion based on the experiment performed    
%            
           

%Bring to same coordinate systeme Fusion and Kuka data
Kuka.enu = convertLocalToEnu(Kuka.trajectory.position);
Kuka.cartesian = convertEnuToCartesian(Kuka.enu, Parameter.center);

%Bring the fused position into Local coordinate system from Kuka
Fusion.enu = convertLocalToEnu(Fusion.position')';
Fusion.cartesian = convertEnuToCartesian(Fusion.enu', Parameter.center);

%DEBUGGING
% visualisationMotion2D(Kuka, Fuse);
% visualisationMotion3D(Kuka,Fuse);

%Plot Ground truth and Fuse solution
compareFusion(Fusion, Kuka, Parameter);


end