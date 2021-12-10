function R_gps = compute_R(simpar)

R_gps = diag([simpar.truth.params.sig_gps_x, simpar.truth.params.sig_gps_y,...
    simpar.truth.params.sig_gps_z].^2);
end

