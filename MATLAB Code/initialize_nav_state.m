function [ xhat ] = initialize_nav_state(simpar,P, x)
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
%% Check for a "reference run"
fnames = fieldnames(simpar.truth.ic);
zero_truth_ic = false(numel(fnames),1);
for i = 1:numel(fnames)
    zero_truth_ic(i) = simpar.truth.ic.(fnames{i}) == 0;
end

%% Initialize navigation states
if all(zero_truth_ic)
    delx_0 = zeros(simpar.states.nxfe,1);
    xhat = injectErrors(truth2nav(x,simpar),delx_0, simpar);
else
    % Cholesky factorization
    [L,p] = chol(P,'lower');
    assert(p == 0, 'Phat_0 is not positive definite');
    
    delx_0 = L * randn(simpar.states.nxfe,1);
    xhat = injectErrors(truth2nav(x,simpar),delx_0, simpar);
end
end
