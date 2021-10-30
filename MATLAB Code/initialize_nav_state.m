function [ xhat ] = initialize_nav_state(simpar, x)
%initialize_nav_state initializes the navigation state vector consistent
%with the initial covariance matrix
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
% [ output_args ] = initialize_nav_state( input_args )

% Author: Isaac Froisland
% Date: 6-Oct-2021

% Consistent with the truth state initialization, you should randomize the
% vehicle states, and initialize any sensor parameters to zero.  An example
% of these calculations are shown below.

% [L_posvelatt,p] = chol(P(simpar.states.ixfe.vehicle,...
%     simpar.states.ixfe.vehicle,1),'lower');
% assert(p == 0, 'Phat_0 is not positive definite');
% delx_0 = zeros(simpar.states.nxfe,1);
% delx_0(simpar.states.ixfe.vehicle,1) = L_posvelatt * ...
%     randn(length(simpar.states.ixfe.vehicle),1);
% xhat = injectErrors(truth2nav(x),delx_0, simpar);
% xhat(simpar.states.ixf.parameter,1) = 0;

xhat = truth2nav(x,simpar);
end
