function [ xhat ] = truth2nav(x_t, simpar)
%truth2nav maps the truth state vector to the navigation state vector
%
% Inputs:
%   x_t = truth state (mixed units)
%
% Outputs
%   x = navigation state (mixed units)
%
% Example Usage
% [ xhat ] = truth2nav( x )

% Author: Randy Christensen
% Date: 21-May-2019 14:17:45
% Reference: 
% Copyright 2019 Utah State University

M = zeros(simpar.states.nxf, simpar.states.nx); 
M(simpar.states.ixf.pos, simpar.states.ix.pos) = eye(length(simpar.states.ix.pos));
M(simpar.states.ixf.vel, simpar.states.ix.vel) = eye(length(simpar.states.ix.vel));
M(simpar.states.ixf.att, simpar.states.ix.att) = eye(length(simpar.states.ix.att));
M(simpar.states.ixf.abias, simpar.states.ix.abias) = eye(length(simpar.states.ix.abias));
M(simpar.states.ixf.gbias, simpar.states.ix.gbias) = eye(length(simpar.states.ix.gbias));
M(simpar.states.ixf.cpos, simpar.states.ix.cpos) = eye(length(simpar.states.ix.cpos));
xhat = M*x_t;
end
