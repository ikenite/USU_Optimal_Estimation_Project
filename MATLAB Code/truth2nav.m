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


xhat = nan(simpar.states.nxf,size(x_t,2));

for i = 1:size(x_t,2)
    xhat(simpar.states.ixf.pos,i) = [x_t(simpar.states.ix.pos_E,i);...
                                   x_t(simpar.states.ix.pos_N,i);...
                                   0];
    xhat(simpar.states.ixf.vel,i) = ...
        [x_t(simpar.states.ix.vel_yb,i)*sin(x_t(simpar.states.ix.head_angle,i));...
         x_t(simpar.states.ix.vel_yb,i)*cos(x_t(simpar.states.ix.head_angle,i));...
         0];

    xhat(simpar.states.ixf.att,i) = [cos(x_t(simpar.states.ix.head_angle,i)/2);...
                                   0;...
                                   0;...
                                   sin(x_t(simpar.states.ix.head_angle,i)/2)];

    xhat(simpar.states.ixf.abias,i) = x_t(simpar.states.ix.abias,i);

    xhat(simpar.states.ixf.gbias,i) = x_t(simpar.states.ix.gbias,i);

    xhat(simpar.states.ixf.cpos,i) = x_t(simpar.states.ix.cpos,i);
end
end
