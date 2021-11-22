function [H_gps] = compute_H(simpar)
%compute_H_example calculates the measurement sensitivity matrix
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
% [ output_args ] = compute_H_example( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:04:33
% Reference: 
% Copyright 2020 Utah State University

H_gps = zeros(3,simpar.states.nxfe);
H_gps(:,simpar.states.ixfe.pos) = eye(3);
end
