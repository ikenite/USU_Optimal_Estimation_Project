function R_pdoa = compute_R(simpar)

T = simpar.general.T_ibc;
A1 = simpar.general.A1_ibc;
A2 = simpar.general.A2_ibc;
Q = simpar.nav.params.sig_pdoa;

R_pdoa = (2/T)*(Q^2/A1^2 + Q^2/A2^2);
end

