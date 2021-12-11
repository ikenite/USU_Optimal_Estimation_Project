function P = update_covariance(P_pre, K, H, R, G, simpar )
%update_covariance updates the covariance matrix

% Author: Randy Christensen
% Date: 31-Aug-2020 16:02:09
% Reference: 
% Copyright 2020 Utah State University

%Don't forget to perform numerical checking and conditioning of covariance
%matrix
I = eye(simpar.states.nxfe);

P = (I - K*H)*P_pre*(I - K*H)' + K*G*R*G'*K';
end
