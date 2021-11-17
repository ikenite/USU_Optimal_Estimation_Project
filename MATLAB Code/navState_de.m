function xhatdot = navState_de(xhat,input)
%navState_de computes the derivative of the nav state
%
% Inputs:
%   xhat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   xhatdot = nav state derivative (mixed units)
%
% Example Usage
% xhatdot = navState_de(xhat,input)

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%% Unpack the inputs
simpar = input.simpar;
omega_tilde = input.omega_tilde;
a_tilde = input.a_tilde;
tau_a = simpar.general.tau_a;
tau_g = simpar.general.tau_g;
b_a_hat = xhat(simpar.states.ixf.abias);
b_g_hat = xhat(simpar.states.ixf.gbias);
g_vec_i = [0; 0; -simpar.general.g];

%% Precalculations
q = xhat(simpar.states.ixf.att);
q = q./norm(q);
nav_dcm = q2tmat(q);

%% Compute individual elements of x_dot
% Time-derivative of position
 xhatdot(simpar.states.ixf.pos) = xhat(simpar.states.ixf.vel);

% Time-derivative of velocity
xhatdot(simpar.states.ixf.vel) = nav_dcm'*(a_tilde - b_a_hat) + g_vec_i;

% Time-derivative of attitude quaternion
w_quat = [0; omega_tilde-b_g_hat];
xhatdot(simpar.states.ixf.att) = (1/2)*qmult(w_quat, q);

% Time-derivative of accel bias
xhatdot(simpar.states.ixf.abias) = -(1/tau_a)*b_a_hat;

% Time-derivative of gyro bias
xhatdot(simpar.states.ixf.gbias) = -(1/tau_g)*b_g_hat;

% Time-derivative of ground circuit position
xhatdot(simpar.states.ixf.cpos) = [0; 0; 0];

% Transpose x_dot for column vector
xhatdot = xhatdot';
end
