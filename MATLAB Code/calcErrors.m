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
del_r = x(simpar.states.ix.pos) - xhat(simpar.states.ixf.pos);
del_v = x(simpar.states.ix.vel) - xhat(simpar.states.ixf.vel);
pre_del_theta = qmult(x(simpar.states.ix.att), qConjugate(xhat(simpar.states.ixf.att)));
del_theta = 2*pre_del_theta([2 3 4]);
del_ba = x(simpar.states.ix.abias) - xhat(simpar.states.ixf.abias);
del_bg = x(simpar.states.ix.gbias) - xhat(simpar.states.ixf.gbias);
del_rc = x(simpar.states.ix.cpos) - xhat(simpar.states.ixf.cpos);

for i=1:m_x
    dele = [del_r; del_v; del_theta; del_ba; del_bg; del_rc];
end
end
