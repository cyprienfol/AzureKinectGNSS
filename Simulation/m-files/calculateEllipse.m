% Calculate the confidence ellipse from DATA with a CONFIDENCE %
%   compareProjection.m 
%
%   See also PLOT, matlab built-in function
%            CALCULATEELLIPSE
%
%   Copyright, Master Thesis:
%   Sensor Fusion of Vision-RTK and Azure Kinect for outfoor AR Applications
%   Cyprien Fol, ETH Zürich.
%

function [r_ellipse, X0, Y0] = calculateEllipse(data, confidence)
% inputs (2) : - data is nx3 dataset that represents the error of
%                projection
%
%              - confidence is a double representing the percentage
%                of confidence, possible values are {95,97,5,99} by
%                default it is 1 sigma (39.7)
%    
% outputs (3): - r_ellipse is a vecor of double representing the ellipse      
%
%              - (X0,Y0) is a pair of double that represents the center of the ellipse 
%


% Calculate the eigenvectors and eigenvalues
covariance = cov(data);
[eigenvec, eigenval ] = eig(covariance);

% Get the index of the largest eigenvector
[largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

% Get the largest eigenvalue
largest_eigenval = max(max(eigenval));

% Get the smallest eigenvector and eigenvalue
if(largest_eigenvec_ind_c == 1)
    smallest_eigenval = max(eigenval(:,2))
    smallest_eigenvec = eigenvec(:,2);
else
    smallest_eigenval = max(eigenval(:,1))
    smallest_eigenvec = eigenvec(1,:);
end

% Calculate the angle between the x-axis and the largest eigenvector
angle = atan2(largest_eigenvec(2), largest_eigenvec(1));

% This angle is between -pi and pi.
% Let's shift it such that the angle is between 0 and 2pi
if(angle < 0)
    angle = angle + 2*pi;
end

% Get the coordinates of the data mean
avg = mean(data);

% Get the 95% confidence interval error ellipse
switch confidence 
    case 95
        chisquare_val= 5.991;
    case 97.5
        chisquare_val= 7.378;
    case 99
        chisquare_val= 9.210;
    otherwise
        chisquare_val = 1;
end
        
    
theta_grid = linspace(0,2*pi);
phi = angle;
X0=avg(1);
Y0=avg(2);
a=sqrt(chisquare_val*largest_eigenval);
b=sqrt(chisquare_val*smallest_eigenval);

% the ellipse in x and y coordinates 
ellipse_x_r  = a*cos( theta_grid );
ellipse_y_r  = b*sin( theta_grid );

%Define a rotation matrix
R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

%let's rotate the ellipse to some angle phi
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
end
