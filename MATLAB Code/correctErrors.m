function [ x_hat_c ] = correctErrors( x_hat, dele, simpar)
%correctState corrects the state vector given an estimated error state
%vector
%
% Inputs:
%   x_hat = estimated state vector (mixed units)
%   delx = error state vector (mixed units)
%
% Outputs
%   x = description (units)
%
% Example Usage
% [ x ] = correctState( x_hat, delx )

% Author: Randy Christensen
% Date: 10-Dec-2018 11:44:28
% Reference:
% Copyright 2018 Utah State University

%Get size of input and verify that it is a single vector
[~,m_x] = size(x_hat);
[~, m_delx] = size(dele);
assert(m_x == m_delx);
x_hat_c = nan(simpar.states.nxf,m_x);
%Correct errors

% Errors have been calculated and need to be removed

% Error correction mapping [Equation (5) in Research Paper]
r = x_hat(simpar.states.ixf.pos) + dele(simpar.states.ixfe.pos);
v = x_hat(simpar.states.ixf.vel) + dele(simpar.states.ixfe.vel);
quat1 = [1;  dele(simpar.states.ixfe.att)/2];
quat2 = x_hat(simpar.states.ixf.att);
q = qmult(quat1, quat2);
q = normalizeQuat(q);
ba = x_hat(simpar.states.ixf.abias) + dele(simpar.states.ixfe.abias);
bg = x_hat(simpar.states.ixf.gbias) + dele(simpar.states.ixfe.gbias);
rc = x_hat(simpar.states.ixf.cpos) + dele(simpar.states.ixfe.cpos);

for i=1:m_x
    x_hat_c = [r; v; q; ba; bg; rc];  
end
end
