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

%TODO: Implement the predicted measurement function (h in Eq. 11 of Debugging Guide)
% Unpack variables
r1_b = [simpar.general.r1_x; simpar.general.r1_y; simpar.general.r1_z]; ...
    % Sensing coil 1 in body frame
r2_b = [simpar.general.r2_x; simpar.general.r2_y; simpar.general.r2_z]; ...
    % Sensing coil 2 in body frame
r_c = x_hat(simpar.states.ixf.cpos); % Ground circuit position in inertial frame
r_b = x_hat(simpar.states.ixf.pos); % Vehicle position in inertial frame
q = x_hat(simpar.states.ixf.att); % Attitude quaternion of vehicle wrt inertial frame
q_conj = qConjugate(q);
f = simpar.general.f; % Carrier frequency [hz]
c = simpar.general.c; % Speed of light in air

% Calculate d_1 and d_2 (distances from sensing coils to ground circuit)
r1_quat = [0; r1_b];
r1_pre = qmult(q_conj, qmult(r1_quat, q));
d_1 = norm(r_c - r_b - r1_pre([2 3 4]));

r2_quat = [0; r2_b];
r2_pre = qmult(q_conj, qmult(r2_quat, q));
d_2 = norm(r_c - r_b - r2_pre([2 3 4]));

% Calculate z_tilde based on measurement model
z_tilde_hat = (2*pi*f/c)*(d_2-d_1);
end
