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
r = x_hat([1 2 3]) + dele([1 2 3]);
v = x_hat([4 5 6]) + dele([4 5 6]);
quat1 = [1;  dele([7 8 9])/2];
quat2 = x_hat([7 8 9 10]);
q = qmult(quat1, quat2);
q = normalizeQuat(q);
ba = x_hat([11 12 13]) + dele([10 11 12]);
bg = x_hat([14 15 16]) + dele([13 14 15]);
rc = x_hat([17 18 19]) + dele([16 17 18]);

for i=1:m_x
    x_hat_c = [r; v; q; ba; bg; rc];  
end
end
