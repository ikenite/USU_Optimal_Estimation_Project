function [H_ibc] = compute_H(xhat, simpar)
%compute_H_example calculates the measurement sensitivity matrix
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
% [ output_args ] = compute_H_example( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:04:33
% Reference: 
% Copyright 2020 Utah State University

%% Unpack inputs
rc_i_hat = xhat(simpar.states.ixf.cpos);
rb_i_hat = xhat(simpar.states.ixf.pos);
r1_b = [simpar.general.r1_x; simpar.general.r1_y; simpar.general.r1_z]; ...
    % Sensing coil 1 in body frame
r2_b = [simpar.general.r2_x; simpar.general.r2_y; simpar.general.r2_z]; ...
    % Sensing coil 2 in body frame
q_hat = xhat(simpar.states.ixf.att);
q_hat = q_hat./norm(q_hat);
T_b_to_i_hat = q2tmat(q_hat);
f = simpar.general.f; % Carrier frequency [hz]
c = simpar.general.c; % Speed of light in air

%% Calculate u-vectors
u_1_hat = (rc_i_hat - rb_i_hat - T_b_to_i_hat*r1_b)...
           /norm(rc_i_hat - rb_i_hat - T_b_to_i_hat*r1_b);
u_2_hat = (rc_i_hat - rb_i_hat - T_b_to_i_hat*r2_b)...
           /norm(rc_i_hat - rb_i_hat - T_b_to_i_hat*r2_b);

%% Calculate components of H_ibc
h1 = (2*pi*f/c)*(u_1_hat' - u_2_hat');
h2 = zeros(1,3);
h3 = (2*pi*f/c)*(u_2_hat'*T_b_to_i_hat*vx(r2_b)...
                 - u_1_hat'*T_b_to_i_hat*vx(r1_b));
h4 = zeros(1,3);
h5 = zeros(1,3);
h6 = (2*pi*f/c)*(u_2_hat' - u_1_hat');

H_ibc = [h1 h2 h3 h4 h5 h6];
end
