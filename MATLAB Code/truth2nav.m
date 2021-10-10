function [ xhat ] = truth2nav(x_t)
%truth2nav maps the truth state vector to the navigation state vector
%
% Inputs:
%   x_t = truth state (mixed units)
%
% Outputs
%   x = navigation state (mixed units)
%
% Example Usage
% [ xhat ] = truth2nav( x )

% Author: Randy Christensen
% Date: 21-May-2019 14:17:45
% Reference: 
% Copyright 2019 Utah State University

xhat = [x_t(1);  % x position of vehicle
        x_t(2);  % y position of vehicle
        x_t(3);  % z position of vehicle
        x_t(4);  % x velocity
        x_t(5);  % y velocity
        x_t(6);  % z velocity
        x_t(7);  % quaternion 1
        x_t(8);  % quaternion 2
        x_t(9);  % quaternion 3
        x_t(10); % quaternion 4
        x_t(15); % x accelerometer bias
        x_t(16); % y accelerometer bias
        x_t(17); % z accelerometer bias
        x_t(18); % x gyroscope bias
        x_t(19); % y gyroscope bias
        x_t(20); % z gyroscope bias
        x_t(21); % x position of coil
        x_t(22); % y position of coil
        x_t(23)];% z position of coil
end
