function [ delx_dot ] = errorState_de( delx, input )
%NAVCOV_DE specifies the differential equation that governs the propagation
%of the covariance of the navigation state estimates

% Unpack inputs
w_vec = input.w_vec;
x_hat = input.x_hat;
y_tilde = input.y_tilde;
simpar = input.simpar;

% Compute state dynamics matrix
Fhat = calc_F(x_hat, y_tilde, simpar);

% Compute process noise coupling matrix
B = calc_B(xhat, simpar);

% Compute delx_dot (Eq. 5 in debugging guide)
delx_dot = Fhat*delx + B*w_vec;
end
