function [ B ] = calc_B(xhat, simpar )
%calc_G Calculates the process noise dynamic coupling matrix
%
% Inputs:
%   xhat = state vector
%   simpar= simulation parameters
%
% Outputs
%   G = process noise dynamic coupling matrix
%
% Example Usage
% [ G ] = calc_G( xhat, simpar )

% Author: Randy Christensen
% Date: 13-May-2020
% Reference: None
% Copyright 2020 Utah State University

%% Unpack the inputs
q_hat_i2b = xhat(simpar.states.ixf.att);
T_hat_b2i = q2tmat(q_hat_i2b)';

%% Compute B
B = zeros(simpar.states.nxfe, 4);
B([4:6],[1:3]) = -T_hat_b2i;
B([7:9],[4:6]) = -eye(3);
B([10:12],[7:9]) = eye(3);
B([13:15],[10:12]) = eye(3);
end
