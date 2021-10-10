function [ xhat ] = initialize_nav_state(simpar)
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

% Populate nav state vector with initial position values
xhat(simpar.states.ixf.pos,1) = [simpar.general.ic.ri_x; ...
                             simpar.general.ic.ri_y; ...
                             simpar.general.ic.ri_z];

% Populate nav state vector with initial velocity values
xhat(simpar.states.ixf.vel,1) = [simpar.general.ic.vi_x; ...
                             simpar.general.ic.vi_y; ...
                             simpar.general.ic.vi_z];
                         
% Populate nav state vector with initial attitude values
xhat(simpar.states.ixf.att,1) = [simpar.general.ic.q_1; ...
                             simpar.general.ic.q_2; ...
                             simpar.general.ic.q_3; ...
                             simpar.general.ic.q_4];

% Populate nav state vector with initial accelerometer bias values
xhat(simpar.states.ixf.abias,1) = [simpar.general.ic.abias_x; ...
                             simpar.general.ic.abias_y; ...
                             simpar.general.ic.abias_z];
                         
% Populate nav state vector with initial gyroscope bias values
xhat(simpar.states.ixf.gbias,1) = [simpar.general.ic.gbias_x; ...
                             simpar.general.ic.gbias_y; ...
                             simpar.general.ic.gbias_z];
                         
% Populate nav state vector with initial coil position values
xhat(simpar.states.ixf.cpos,1) = [simpar.general.ic.rc_x; ...
                             simpar.general.ic.rc_y; ...
                             simpar.general.ic.rc_z];
end
