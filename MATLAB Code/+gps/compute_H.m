function [H_gps] = compute_H(x_hat, simpar)
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

r_gps_b = [simpar.general.r_gps_x; simpar.general.r_gps_y; simpar.general.r_gps_z]; ...
    % GPS in body frame
q_hat = x_hat(simpar.states.ixf.att); 
q_hat = q_hat./norm(q_hat);
T_b_to_i_hat = q2tmat(q_hat)';

H_gps = zeros(3,simpar.states.nxfe);
H_gps(:,simpar.states.ixfe.pos) = eye(3);
H_gps(:,simpar.states.ixfe.att) = -T_b_to_i_hat*vx(r_gps_b);
end
