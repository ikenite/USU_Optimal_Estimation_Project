function [omega] = calc_omega(x, simpar)

phi = x(simpar.states.ix.st_angle);
vy = x(simpar.states.ix.vel(2));
omega =[0; 0; vy/simpar.general.L*tan(phi)];
end

