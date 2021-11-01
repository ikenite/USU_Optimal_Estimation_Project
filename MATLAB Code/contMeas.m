function [ ytilde ] = contMeas(x, a_y, simpar)
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
%TODO: Implement realization of noise for accels and gyros
n_a = zeros(3,1);
n_g = zeros(3,1);

omega = calc_omega(x,simpar);
a = calc_accel(a_y,x,simpar);

a_tilde = a + b_a + n_a;
omega_tilde = omega + b_g + n_g;
ytilde = [a_tilde; omega_tilde];
end
