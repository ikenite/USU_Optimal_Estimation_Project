function [ xhat_err ] = injectErrors( xhat_true, dele, simpar )
%injectErrors injects errors into the state estimates
%
% Inputs:
%   xhat_true = true navigation state vector (mixed units)
%   delx = error state vector (mixed units)
%
% Outputs
%   xhat_err = estimated state vector(units)
%
% Example Usage
% [ xhat_err ] = injectErrors( xhat_true, delx )

% Author: Randy Christensen
% Date: 10-Dec-2018 11:54:42
% Reference: 
% Copyright 2018 Utah State University

%Get size of inputs
[~,m_x] = size(xhat_true);
[~, m_delx] = size(dele);
assert(m_x == m_delx);
%Inject errors
xhat_err = zeros(simpar.states.nxf,m_x);

% Error injection mapping [Equation (6) in Research Paper]
r_hat = xhat_true([1 2 3]) - dele([1 2 3]);
v_hat = xhat_true([4 5 6]) - dele([4 5 6]);
quat1 = [1;  -dele([7 8 9])/2];
quat2 = xhat_true([7 8 9 10]);
q_hat = qmult(quat1, quat2);
%q_hat = normalizeQuat(q_hat); % May not want this...
ba_hat = xhat_true([11 12 13]) - dele([10 11 12]);
bg_hat = xhat_true([14 15 16]) - dele([13 14 15]);
rc_hat = xhat_true([17 18 19]) - dele([16 17 18]);

for i=1:m_x
    xhat_err = [r_hat; v_hat; q_hat; ba_hat; bg_hat; rc_hat];
end
end
