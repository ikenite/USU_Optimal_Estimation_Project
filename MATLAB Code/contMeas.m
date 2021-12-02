function [ ytilde ] = contMeas(x, input, simpar)
%contInertialMeas synthesizes noise measurements used to propagate the
%navigation state
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
% [ output_args ] = contInertialMeas( input_args )

% Author: 
% Date: 31-Aug-2020 15:46:59
% Reference: 
% Copyright 2020 Utah State University

% Unpack variables
b_a = x(simpar.states.ix.abias);
b_g = x(simpar.states.ix.gbias);
g_vec_b = [0; 0; -simpar.general.g];
a_y = input.a_y;
n_a = input.w_vec([1:3]);
n_g = input.w_vec([4:6]);

omega = calc_omega(x,simpar);
a = calc_accel(a_y,x,simpar);

a_tilde = a - g_vec_b + b_a + n_a;
omega_tilde = omega + b_g + n_g;
ytilde = [a_tilde; omega_tilde];
end
