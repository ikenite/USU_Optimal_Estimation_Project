function K = compute_Kalman_gain(H, P, R, G)
%compute_Kalman_gain calculates the Kalman gain

% Author: Randy Christensen
% Date: 31-Aug-2020 16:01:57
% Reference: 
% Copyright 2020 Utah State University

K = P*H'/(H*P*H' + G*R*G');
end
