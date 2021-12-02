function [z_tilde_hat] = predict_measurement(x_hat, simpar)
%predict_measurement_example predicts the discrete measurement
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
% [ output_args ] = predict_measurement_example( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:00:53
% Reference: 
% Copyright 2020 Utah State University

% Implement the predicted measurement function (h in Eq. 11 of Debugging Guide)
% Unpack variables
r_b_i_hat = x_hat(simpar.states.ixf.pos); % Vehicle position in inertial frame
r_gps_b = [simpar.general.r_gps_x; simpar.general.r_gps_y; simpar.general.r_gps_z]; ...
    % GPS in body frame
q_hat = x_hat(simpar.states.ixf.att); 
q_hat = q_hat./norm(q_hat);
T_b_to_i_hat = q2tmat(q_hat)';

% Calculate z_tilde based on measurement model
z_tilde_hat = r_b_i_hat + T_b_to_i_hat*r_gps_b;
end
