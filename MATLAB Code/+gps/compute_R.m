function R_gps = compute_R(simpar)

R_gps = diag([simpar.nav.params.sig_gps_x, simpar.nav.params.sig_gps_y,...
    simpar.nav.params.sig_gps_z].^2);
end

