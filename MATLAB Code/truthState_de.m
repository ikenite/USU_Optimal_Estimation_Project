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
tau_a = simpar.general.tau_a;
tau_g = simpar.general.tau_g;
w_a = input.w([1 2 3]);
w_g = input.w([4 5 6]);

%% Compute individual elements of x_dot
% Time-derivative of position
xdot(simpar.states.ix.pos) = x(simpar.states.ix.vel);

% Time-derivative of velocity
q_conj = qConjugate(x(simpar.states.ix.att));
q = x(simpar.states.ix.att);
a_quat = [0; 0; a_y; 0];
v_dot_pre = qmult(q_conj, qmult(a_quat, q));
xdot(simpar.states.ix.vel) = v_dot_pre([2 3 4]);

% Time-derivative of attitude quaternion
w_quat = [0; calc_omega(x, simpar)];
xdot(simpar.states.ix.att) = (1/2)*qmult(w_quat, q);
 
% Time-derivative of steering angle
xdot(simpar.states.ix.st_angle) = xi;

% Time-derivative of accel bias
xdot(simpar.states.ix.abias) = -(1/tau_a)*x(simpar.states.ix.abias) + w_a;

% Time-derivative of gyro bias
xdot(simpar.states.ix.gbias) = -(1/tau_g)*x(simpar.states.ix.gbias) + w_g;

% Time-derivative of ground circuit position
xdot(simpar.states.ix.cpos) = [0; 0; 0];

% Transpose x_dot for column vector
xdot = xdot';
end