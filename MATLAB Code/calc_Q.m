function [ Q ] = calc_Q(simpar )
%calc_Q Calculates the process noise power spectral density
%
% Inputs:
%   xhat = state vector
%   simpar= simulation parameters
%
% Outputs
%   Q = process noise dynamic coupling matrix
%
% Example Usage
% [ Q ] = calc_Q( xhat, simpar )

% Author: Randy Christensen
% Date: 13-May-2020
% Reference: None
% Copyright 2020 Utah State University

%% Unpack the inputs

%% Calcs
Q_a_vrw = simpar.truth.params.vrw^2;
Q_g_arw = simpar.truth.params.arw^2;
Q_a = 2*simpar.truth.params.sig_accel_ss^2/simpar.general.tau_a;
Q_g = 2*simpar.truth.params.sig_gyro_ss^2/simpar.general.tau_g;

Q_a_vrw_matrix = diag([Q_a_vrw; Q_a_vrw; Q_a_vrw]);
Q_g_arw_matrix = diag([Q_g_arw; Q_g_arw; Q_g_arw]);
Q_a_matrix = diag([Q_a; Q_a; Q_a]);
Q_g_matrix = diag([Q_g; Q_g; Q_g]);

%% Assign Q
Q = blkdiag(Q_a_vrw_matrix, Q_g_arw_matrix, Q_a_matrix, Q_g_matrix);
end
