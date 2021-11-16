function [accel] = calcAccel(a_y, x, simpar)

phi = x(simpar.states.ix.st_angle);
vy = x(simpar.states.ix.vel_yb);
L = simpar.general.L;
g = simpar.general.g;

accel = [-vy^2*tan(phi)/L; a_y; g];
end

