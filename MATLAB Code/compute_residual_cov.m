function P_res = compute_residual_cov(H, P ,R )
%compute_residual_cov calculates the covariance of the residual

% Author: Randy Christensen
% Date: 31-Aug-2020 16:01:44
% Reference: 
% Copyright 2020 Utah State University

P_res = H*P*H' + R;
end
