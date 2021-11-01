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
phi = x(simpar.states.ix.st_angle);
psi = x(simpar.states.ix.head_angle);
vel_yb = x(simpar.states.ix.vel_yb);
L = simpar.general.L;

%% Compute individual elements of x_dot
% Time-derivative of position values
xdot(simpar.states.ix.pos_E) = vel_yb*sin(psi);
xdot(simpar.states.ix.pos_N) = vel_yb*cos(psi);

% Time-derivative of velocity
xdot(simpar.states.ix.vel_yb) = a_y;

% Time-derivative of heading angle
xdot(simpar.states.ix.head_angle) = vel_yb*tan(phi)/L;

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