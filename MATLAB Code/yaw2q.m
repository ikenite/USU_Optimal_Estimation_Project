function [q] = yaw2q(psi)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%DCM from heading angle (psi)
R = [cos(psi) sin(psi) 0;
    -sin(psi) cos(psi) 0;
        0        0     1];
q = dcm2quat(R);    
end

