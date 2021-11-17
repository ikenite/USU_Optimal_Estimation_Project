function checkErrorPropagation( x, xhat, delx_linear, simpar,i)
%checkErrorPropagation checks the linear propagation of errors
% Inputs:
%   x = truth state
%   xhat = estimated state
%   delx_linear = errors propagated using linear model
%
% Example Usage
% checkErrorPropagation( x, xhat, delx_linear)

% Author: Randy Christensen
% Date: 25-Jan-2019 12:27:36
% Reference: 
% Copyright 2018 Utah State University

delx_nonlinear = calcErrors(xhat, x, simpar);
propError = delx_linear - delx_nonlinear;
results = table(delx_linear, delx_nonlinear, propError);
disp(results)

if simpar.general.errorPropTestEnableCont
    hold on
    scatter(i, delx_nonlinear(2),'blue','o')
    scatter(i, delx_linear(2),'red','*')
    legend('delx nonlinear', 'delx linear')
end
end
















