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
dele = nan(simpar.states.nxfe,m_x);

% x_hat now has errors injected in

% Estimation error mapping
for i=1:size(x,2)
    dele(simpar.states.ixfe.pos,i) = x(simpar.states.ixf.pos,i) - xhat(simpar.states.ixf.pos,i);
    dele(simpar.states.ixfe.vel,i) = x(simpar.states.ixf.vel,i) - xhat(simpar.states.ixf.vel,i);
    pre_del_theta = qmult(x(simpar.states.ixf.att,i), qConjugate(xhat(simpar.states.ixf.att,i)));
    dele(simpar.states.ixfe.att,i) = 2*pre_del_theta([2 3 4]);
    dele(simpar.states.ixfe.abias,i) = x(simpar.states.ixf.abias,i) - xhat(simpar.states.ixf.abias,i);
    dele(simpar.states.ixfe.gbias,i) = x(simpar.states.ixf.gbias,i) - xhat(simpar.states.ixf.gbias,i);
    dele(simpar.states.ixfe.cpos,i) = x(simpar.states.ixf.cpos,i) - xhat(simpar.states.ixf.cpos,i);
end

end
