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
b_a = xhat(simpar.states.ixf.abias);
b_g = xhat(simpar.states.ixf.gbias);
g_vec = [0; 0; simpar.general.g];

%% Compute individual elements of x_dot
% Time-derivative of position
xhatdot(simpar.states.ixf.pos) = xhat(simpar.states.ixf.vel);

% Time-derivative of velocity
q_conj = qConjugate(xhat(simpar.states.ixf.att));
q = xhat(simpar.states.ixf.att);
a_quat = [0; a_tilde-b_a];
v_dot_pre = qmult(q, qmult(a_quat, q_conj));
xhatdot(simpar.states.ixf.vel) = v_dot_pre([2 3 4]) - g_vec;

% Time-derivative of attitude quaternion
w_quat = [0; omega_tilde-b_g];
xhatdot(simpar.states.ixf.att) = (1/2)*qmult(w_quat, q);

% Time-derivative of accel bias
xhatdot(simpar.states.ixf.abias) = -(1/tau_a)*xhat(simpar.states.ixf.abias);

% Time-derivative of gyro bias
xhatdot(simpar.states.ixf.gbias) = -(1/tau_g)*xhat(simpar.states.ixf.gbias);

% Time-derivative of ground circuit position
xhatdot(simpar.states.ixf.cpos) = [0; 0; 0];

% Transpose x_dot for column vector
xhatdot = xhatdot';
end
