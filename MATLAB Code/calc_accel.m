function [accel] = calcAccel(a_y, x, simpar)

phi = x(simpar.states.ix.st_angle);
vy = x(simpar.states.ix.vel_yb);
L = simpar.general.L;

accel = [-vy^2*tan(phi)/L; a_y; 0];
end

