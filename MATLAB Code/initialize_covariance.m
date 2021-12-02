function P = initialize_covariance( simpar )
%initialize_covariance computes the initial covariance matrix
%
% Inputs:
%   Input1 = description (units)
%   Input2 = description (units)
%
% Outputs
%   Output1 = description (units)
%   Output2 = description (units)
%
% Example Usage
% [ output_args ] = initialize_covariance( input_args )

% Author: 
% Date: 31-Aug-2020 15:46:59
% Reference: 
% Copyright 2020 Utah State University

%% Define individual covariances
P_pos = diag([simpar.nav.ic.sig_r_E, simpar.nav.ic.sig_r_N, ...
    simpar.nav.ic.sig_r_U].^2);
P_vel = diag([simpar.nav.ic.sig_v_x, simpar.nav.ic.sig_v_y, ...
    simpar.nav.ic.sig_v_z].^2);
P_att = diag([simpar.nav.ic.sig_th_x, simpar.nav.ic.sig_th_y, ...
    simpar.nav.ic.sig_th_z].^2);
P_accel = diag([simpar.nav.ic.sig_accel_x, simpar.nav.ic.sig_accel_y, ...
    simpar.nav.ic.sig_accel_z].^2);
P_gyro = diag([simpar.nav.ic.sig_gyro_x, simpar.nav.ic.sig_gyro_y, ...
    simpar.nav.ic.sig_gyro_z].^2);
P_cpos = diag([simpar.nav.ic.sig_r_c_E, simpar.nav.ic.sig_r_c_N, ...
    simpar.nav.ic.sig_r_c_U].^2);

%% Define covariance matrix
P = zeros(simpar.states.nxe, simpar.states.nxe);
P(simpar.states.ixfe.pos, simpar.states.ixfe.pos) = P_pos;
P(simpar.states.ixfe.vel, simpar.states.ixfe.vel) = P_vel;
P(simpar.states.ixfe.att, simpar.states.ixfe.att) = P_att;
P(simpar.states.ixfe.abias, simpar.states.ixfe.abias) = P_accel;
P(simpar.states.ixfe.gbias, simpar.states.ixfe.gbias) = P_gyro;
P(simpar.states.ixfe.cpos, simpar.states.ixfe.cpos) = P_cpos;
end
