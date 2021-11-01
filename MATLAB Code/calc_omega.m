function [omega] = calc_omega(x, simpar)

phi = x(simpar.states.ix.st_angle);
vy = x(simpar.states.ix.vel_yb);
L = simpar.general.L;
omega =[0; 0; vy/L*tan(phi)];
end

