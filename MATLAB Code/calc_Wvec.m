function [w_vec] = calc_Wvec(simpar)

if simpar.general.noiseOn == true
    % Define PSD of process noise for accel and gyro bias
    Q_a = 2*simpar.truth.params.sig_accel_ss^2/simpar.general.tau_a;
    Q_g = 2*simpar.truth.params.sig_gyro_ss^2/simpar.general.tau_g;
    
    % Synthesize process noise
    w_a = sqrt(Q_a/simpar.general.dt)*randn(3,1);
    w_g = sqrt(Q_g/simpar.general.dt)*randn(3,1);
    
    % Synthesize "measurement" process noise
    Q_a_vrw = simpar.truth.params.vrw^2;
    n_a = sqrt(Q_a_vrw/simpar.general.dt)*randn(3,1);
    Q_g_arw = simpar.truth.params.arw^2;
    n_g = sqrt(Q_g_arw/simpar.general.dt)*randn(3,1);
    
    % Package up noise vector
    w_vec = [n_a; n_g; w_a; w_g];
else
    w_vec = zeros(12,1);
end

end

