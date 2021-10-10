function [ dele ] = calcErrors( xhat, x, simpar )
%calcErrors computes estimation errors
%
% Inputs:
%   x_hat = estimated state vector(mixed units)
%   x = state vector (mixed units)
%
% Outputs
%   dele = estimation error state vector (mixed units)
%
% Example Usage
% [ dele ] = calcErrors( x_hat, x )

% Author: Randy Christensen
% Date: 21-May-2019 13:43:16
% Reference:
% Copyright 2019 Utah State University

%Get size of input and verify that it is a single vector
[~,m_x] = size(x);
[~, m_xhat] = size(xhat);
assert(m_x == m_xhat);
xhat_true = truth2nav(x);
dele = nan(simpar.states.nxfe,m_x);

% x_hat now has errors injected in
% Estimation error mapping [Equation (7) in Research Paper]
del_r = x([1 2 3]) - xhat([1 2 3]);
del_v = x([4 5 6]) - xhat([4 5 6]);
pre_del_theta = qmult(x([7 8 9 10]), qConjugate(xhat([7 8 9 10])));
del_theta = 2*pre_del_theta([2 3 4]);
del_ba = x([15 16 17]) - xhat([11 12 13]);
del_bg = x([18 19 20]) - xhat([14 15 16]);
del_rc = x([21 22 23]) - xhat([17 18 19]);

for i=1:m_x
    dele = [del_r; del_v; del_theta; del_ba; del_bg; del_rc];
end
end
