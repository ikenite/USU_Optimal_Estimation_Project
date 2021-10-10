function [ x ] = initialize_truth_state(simpar)
%initialize_truth_state initialize the truth state vector consistent with
%the initial covariance matrix
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
% [ output_args ] = initialize_truth_state( input_args )

% Author: Isaac Froisland
% Date: 6-Oct-2021

% In the initialization of the truth and navigation states, you need only
% ensure that the estimation error is consistent with the initial
% covariance matrix.  One realistic way to do this is to set the true 
% vehicle states to the same thing every time, and randomize any sensor 
% parameters.

% Initialize vector with number of components in truth state vector
x = zeros(simpar.states.nx,1);

% Populate truth state vector with initial position values
x(simpar.states.ix.pos,1) = [simpar.general.ic.ri_x; ...
                             simpar.general.ic.ri_y; ...
                             simpar.general.ic.ri_z];

% Populate truth state vector with initial velocity values
x(simpar.states.ix.vel,1) = [simpar.general.ic.vi_x; ...
                             simpar.general.ic.vi_y; ...
                             simpar.general.ic.vi_z];
                         
% Populate truth state vector with initial attitude values
x(simpar.states.ix.att,1) = [simpar.general.ic.q_1; ...
                             simpar.general.ic.q_2; ...
                             simpar.general.ic.q_3; ...
                             simpar.general.ic.q_4];
                        
% Populate truth state vector with initial angular rate values
x(simpar.states.ix.ang_rate,1) = [simpar.general.ic.ang_rate_x; ...
                             simpar.general.ic.ang_rate_y; ...
                             simpar.general.ic.ang_rate_z];
                         
% Populate truth state vector with initial steering angle value
x(simpar.states.ix.st_angle,1) = simpar.general.ic.st_angle;

% Populate truth state vector with initial accelerometer bias values
x(simpar.states.ix.abias,1) = [simpar.general.ic.abias_x; ...
                             simpar.general.ic.abias_y; ...
                             simpar.general.ic.abias_z];
                         
% Populate truth state vector with initial gyroscope bias values
x(simpar.states.ix.gbias,1) = [simpar.general.ic.gbias_x; ...
                             simpar.general.ic.gbias_y; ...
                             simpar.general.ic.gbias_z];
                         
% Populate truth state vector with initial coil position values
x(simpar.states.ix.cpos,1) = [simpar.general.ic.rc_x; ...
                             simpar.general.ic.rc_y; ...
                             simpar.general.ic.rc_z];

end
