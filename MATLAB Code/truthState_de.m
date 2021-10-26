function xdot = truthState_de( x, input)
%truthState_de computes the derivative of the truth state
%
% Inputs:
%   x = truth state (mixed units)
%   u = input (mixed units)
%
% Outputs
%   xdot = truth state derivative (mixed units)
%
% Example Usage
% xdot = truthState_de( x, input)

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%% Unpack the inputs
simpar = input.simpar;
a_y = input.u(1);
xi = input.u(2);
phi = x(simpar.states.ix.st_angle);
v_y = x(simpar.states.ix.vel(2));
L   = simpar.general.L;
tau_a = simpar.general.tau_a;
tau_g = simpar.general.tau_g;
w_a = input.w(1);
w_g = input.w(2);
%% Compute individual elements of x_dot
r_dot = x(simpar.states.ix.vel);

q_conj = qConjugate(x(simpar.states.ix.att));
q = x(simpar.states.ix.att);
a_quat = [0; 0; 0; a_y];
v_dot_pre = qmult(q_conj, qmult(a_quat, q));
v_dot = v_dot_pre([2 3 4]);

w_quat = [0; 0; 0; (v_y/L)*tan(phi)];
q_dot = (1/2)*qmult(w_quat, q);
 
phi_dot = xi;

abias_dot = -(1/tau_a)*x(simpar.states.ix.abias) + w_a;

gbias_dot = -(1/tau_g)*x(simpar.states.ix.gbias) + w_g;
rc_dot = [0; 0; 0];

%% Assign to output
xdot = [r_dot; v_dot; q_dot; phi_dot; abias_dot; gbias_dot; rc_dot];
end