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
r_hat = xhat_true(simpar.states.ixf.pos) - dele(simpar.states.ixfe.pos);
v_hat = xhat_true(simpar.states.ixf.vel) - dele(simpar.states.ixfe.vel);
quat1 = [1;  -dele(simpar.states.ixfe.att)/2];
quat2 = xhat_true(simpar.states.ixf.att);
q_hat = qmult(quat1, quat2);
ba_hat = xhat_true(simpar.states.ixf.abias) - dele(simpar.states.ixfe.abias);
bg_hat = xhat_true(simpar.states.ixf.gbias) - dele(simpar.states.ixfe.gbias);
rc_hat = xhat_true(simpar.states.ixf.cpos) - dele(simpar.states.ixfe.cpos);

for i=1:m_x
    xhat_err = [r_hat; v_hat; q_hat; ba_hat; bg_hat; rc_hat];
end
end
