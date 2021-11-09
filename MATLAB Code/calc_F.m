function [ Fhat ] = calc_F( xhat, ytilde, simpar )
%calc_F computes the dynamics coupling matrix
%
% Inputs:
%   xhat = state vector
%   ytilde = continuous measurements
%   simpar = simulation parameters
%
% Outputs
%   Fhat = state dynamics matrix
%
% Example Usage
% [ Fhat ] = calc_F( xhat, ytilde, simpar )

% Author: Randy Christensen
% Date: 13-May-2020
% Reference: None
% Copyright 2020 Utah State University

%% Unpack the inputs
omega_tilde = ytilde([1 2 3],:);
a_tilde = ytilde([4 5 6],:);
tau_a = simpar.general.tau_a;
tau_g = simpar.general.tau_g;
q_hat = xhat(simpar.states.ixf.att);
b_a_hat = xhat(simpar.states.ixf.abias);
b_g_hat = xhat(simpar.states.ixf.gbias);
omega_hat = omega_tilde - b_g_hat;

%% Compute Fhat
Fhat = zeros(simpar.states.nxfe, simpar.states.nxfe);
Fhat(simpar.states.ixfe.pos, simpar.states.ixfe.vel) = eye(3);
Fhat(simpar.states.ixfe.vel, simpar.states.ixfe.att) = ...
    vx(-q2tmat(q_hat)*(a_tilde - b_a_hat));
Fhat(simpar.states.ixfe.vel, simpar.states.ixfe.abias) = -q2tmat(q_hat);
Fhat(simpar.states.ixfe.att, simpar.states.ixfe.att) = -vx(omega_hat);
Fhat(simpar.states.ixfe.att, simpar.states.ixfe.gbias) = -eye(3);
Fhat(simpar.states.ixfe.abias, simpar.states.ixfe.abias) = -(1\tau_a)*eye(3);
Fhat(simpar.states.ixfe.gbias, simpar.states.ixfe.gbias) = -(1\tau_g)*eye(3);
end