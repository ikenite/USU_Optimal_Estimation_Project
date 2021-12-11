function [ z_tilde ] = synthesize_measurement(x, simpar)
%synthesize_measurement_example synthesizes the discrete measurement
%corrupted by noise
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
% [ output_args ] = synthesize_measurement_example( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:00:48
% Reference: 
% Copyright 2020 Utah State University

% Unpack variables
r_b_i = x(simpar.states.ixf.pos); % Vehicle position in inertial frame
r_gps_b = [simpar.general.r_gps_x; simpar.general.r_gps_y; simpar.general.r_gps_z]; ...
    % GPS in body frame
q = x(simpar.states.ixf.att); 
q = q./norm(q);
T_b_to_i = q2tmat(q)';
sig_gps = diag([simpar.truth.params.sig_gps_x, simpar.truth.params.sig_gps_y,...
    simpar.truth.params.sig_gps_z]);

%Synthesize noise
v_gps = sig_gps*randn(3,1);

% Calculate z_tilde based on measurement model
z_tilde = r_b_i + T_b_to_i*r_gps_b + v_gps;
end
