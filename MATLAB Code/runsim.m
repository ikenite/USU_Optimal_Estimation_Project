function [ traj ] = runsim( simpar, verbose, seed)
rng(seed);
%RUNSIM Runs a single trajectory given the parameters in simparams
tic;
%% Prelims
%Derive the number of steps in the simulation and the time
nstep = ceil(simpar.general.tsim/simpar.general.dt + 1);
nstep_ibc_aid = ceil(simpar.general.tsim/simpar.general.dt_kalmanUpdate_ibc);
nstep_gps_aid = ceil(simpar.general.tsim/simpar.general.dt_kalmanUpdate_gps);
t = (0:nstep-1)'*simpar.general.dt;
t_kalman_ibc = (0:nstep_ibc_aid)'.*simpar.general.dt_kalmanUpdate_ibc;
t_kalman_gps = (0:nstep_gps_aid)'.*simpar.general.dt_kalmanUpdate_gps;
nstep_ibc_aid = length(t_kalman_ibc);
nstep_gps_aid = length(t_kalman_gps);
%% Pre-allocate buffers for saving data
% Truth, navigation, and error state buffers
x_buff          = zeros(simpar.states.nx,nstep);
xhat_buff       = zeros(simpar.states.nxf,nstep);
delx_buff       = zeros(simpar.states.nxfe,nstep);
% Navigation covariance buffer
P_buff       = zeros(simpar.states.nxfe,simpar.states.nxfe,nstep);
% Continuous measurement buffer
ytilde_buff     = zeros(simpar.general.n_inertialMeas,nstep);
% Residual buffers
res_gps     = zeros(3,nstep_gps_aid);
resCov_gps  = zeros(3,3,nstep_gps_aid);
K_gps_buff  = zeros(simpar.states.nxfe,3,nstep_gps_aid);
res_ibc     = zeros(1,nstep_ibc_aid);
resCov_ibc  = zeros(3,3,nstep_ibc_aid);
K_ibc_buff  = zeros(simpar.states.nxfe,3,nstep_ibc_aid);
% Discrete measurement buffers
ztilde_gps_buff = zeros(3,nstep_gps_aid);
ztildehat_gps_buff = zeros(3,nstep_gps_aid);
ztilde_ibc_buff = zeros(1,nstep_ibc_aid);
ztildehat_ibc_buff = zeros(1,nstep_ibc_aid);
%% Initialize the navigation covariance matrix
P_buff(:,:,1) = initialize_covariance(simpar);
%% Initialize the truth state vector
x_buff(:,1) = initialize_truth_state(simpar);
%% Initialize the navigation state vector
xhat_buff(:,1) = initialize_nav_state(simpar,P_buff(:,:,1), x_buff(:,1));
%% System Inputs
% Vehicle begins with initial velocity and zero steering angle oriented North.
% A periodic input causes the vehicle to follow a sinusoid-like path, passing
% the ground coil along the way.
input_time_scalar = 1/simpar.general.dt;
steering_rate = 5*pi/180; %[deg/s] to [rad/s]
steering_T = 1; % [1/hz] --> T = 1/f
steering_frequency = (2*pi)*(1/steering_T); % omega = 2pi*f 
c_factor = -0.0005;

a_y = 0*ones(nstep,1);

xi = zeros(nstep,1);
xi_t_vec_1 = t(1:steering_T*input_time_scalar/2+1);
xi(1:steering_T*input_time_scalar/2+1) = ...
    4*steering_frequency*steering_rate.*cos(2*steering_frequency.*(xi_t_vec_1));
    
for i = steering_T*input_time_scalar/2+2:nstep
    if mod(t(i)-steering_T/2,2*steering_T) < steering_T % (t - T/2)%2T < T
        xi(i) = steering_frequency*steering_rate*cos(steering_frequency*(t(i)+c_factor));
    else
        xi(i) = -steering_frequency*steering_rate*cos(steering_frequency*(t(i)+c_factor));
    end
end

system_input.a_y = a_y;
system_input.xi = xi;
%% Miscellaneous calcs
% Synthesize noise vector at t_n-1 = t_1
w_vec = calc_Wvec(simpar);
%w_vec = [n_a; n_g; w_a; w_g];
%Indices (1:3)(4:6)(7:9)(10:12)

% Synthesize continuous sensor data at t_n-1 = t_1
input_ytilde.a_y = a_y(1);
input_ytilde.w_vec = w_vec;
ytilde_buff(:,1) = contMeas(x_buff(:,1), input_ytilde, simpar);

%Compute process noise PSD matrix
Q = calc_Q(simpar);

%Initialize the measurement counters
k_gps = 1;
k_ibc = 1;
%Check that the error injection, calculation, and removal are all
%consistent if the simpar.general.checkErrDefConstEnable is enabled.
if simpar.general.checkErrDefConstEnable
    checkErrorDefConsistency(xhat_buff(:,1), truth2nav(x_buff(:,1),simpar), simpar)
end
%Inject errors if the simpar.general.errorPropTestEnable flag is enabled
if simpar.general.errorPropTestEnable
    fnames = fieldnames(simpar.errorInjection);
    for i=1:length(fnames)
        delx_buff(i,1) = simpar.errorInjection.(fnames{i});
    end
    xhat_buff(:,1) = injectErrors(truth2nav(x_buff(:,1),simpar), delx_buff(:,1), simpar);
end
%% Loop over each time step in the simulation
for i=2:nstep
    % Propagate truth states to t_n
    %   Realize a sample of process noise (don't forget to scale Q by 1/dt!)
    %   Define any inputs to the truth state DE
    %   Perform one step of RK4 integration
    
    % Synthesize noise vector
    w_vec = calc_Wvec(simpar); 
    
    % Propagate truth state forward one dt
    input_truth.u = [a_y(i); xi(i)];
    input_truth.w_vec = w_vec; 
    input_truth.simpar = simpar;
    x_buff(:,i) = rk4('truthState_de',x_buff(:,i-1), input_truth,...
        simpar.general.dt);
    
    % Synthesize continuous sensor data at t_n
    input_ytilde.a_y = a_y(i);
    input_ytilde.w_vec = w_vec;
    ytilde_buff(:,i) = contMeas(x_buff(:,i), input_ytilde, simpar);
    
    % Propagate navigation states to t_n using sensor data from t_n-1
    %   Assign inputs to the navigation state DE
    %   Perform one step of RK4 integration
    input_nav.a_tilde = ytilde_buff([1 2 3],i);
    input_nav.omega_tilde = ytilde_buff([4 5 6],i);
    input_nav.simpar = simpar;
    xhat_buff(:,i) = rk4('navState_de', xhat_buff(:,i-1), input_nav, ...
        simpar.general.dt);
    
    % Propagate the covariance to t_n
    input_cov.ytilde = ytilde_buff(:,i);
    input_cov.simpar = simpar;
    input_cov.Q = Q;
    input_cov.xhat = xhat_buff(:,i);
    P_buff(:,:,i) = rk4('navCov_de', P_buff(:,:,i-1), input_cov, ...
        simpar.general.dt);
    
    % Propagate the error state from tn-1 to tn if errorPropTestEnable == 1
    if simpar.general.errorPropTestEnable
        input_delx.x_hat = xhat_buff(:,i-1);
        input_delx.y_tilde = ytilde_buff(:,i);
        input_delx.w_vec = w_vec;
        input_delx.simpar = simpar;
        delx_buff(:,i) = rk4('errorState_de', delx_buff(:,i-1), ...
            input_delx, simpar.general.dt);
        if simpar.general.errorPropTestEnableCont
            checkErrorPropagation(truth2nav(x_buff(:,i),simpar), xhat_buff(:,i),...
                delx_buff(:,i), simpar,i);
        end
    end
    
    % If GPS discrete measurements are available, perform a Kalman update
    if abs(t(i)-t_kalman_gps(k_gps+1)) < simpar.general.dt*0.01
        %   Check error state propagation if simpar.general.errorPropTestEnable = true
        if simpar.general.errorPropTestEnable
            checkErrorPropagation(truth2nav(x_buff(:,i),simpar), xhat_buff(:,i),...
                delx_buff(:,i), simpar,i);
        end
        %Adjust the Kalman update index
        k_gps = k_gps + 1;
        %   For each available measurement
        %       Synthesize the noisy measurement, ztilde
        %       Predict the measurement, ztildehat
        %       Compute the measurement sensitivity matrix, H
        %       If simpar.general.measLinerizationCheckEnable == true
        %           Check measurement linearization
        %       Compute and save the residual
        %       Compute and save the residual covariance
        %       Compute and save the Kalman gain, K
        %       Estimate the error state vector
        %       Update and save the covariance matrix
        %       Correct and save the navigation states
        if simpar.general.process_GPS_enable
            ztilde_gps_buff(:,k_gps) = gps.synthesize_measurement(truth2nav(x_buff(:,i), simpar), simpar);
            ztildehat_gps_buff(:,k_gps) = gps.predict_measurement(xhat_buff(:,i), simpar);
            H_gps = gps.compute_H(xhat_buff(:,i),simpar);
            gps.validate_linearization(x_buff(:,i), simpar);
            res_gps(:,k_gps) = gps.compute_residual(ztilde_gps_buff(:,k_gps), ztildehat_gps_buff(:,k_gps));
            
            %         resCov_gps(:,k) = compute_residual_cov();
            %         K_gps_buff(:,:,k) = compute_Kalman_gain();
            %         del_x = estimate_error_state_vector();
            %         P_buff(:,:,k) = update_covariance();
            %         xhat_buff(:,i) = correctErrors();
        end
    end
    % If IBC discrete measurements are available, perform a Kalman update
    if abs(t(i)-t_kalman_ibc(k_ibc+1)) < simpar.general.dt*0.01
        %   Check error state propagation if simpar.general.errorPropTestEnable = true
        if simpar.general.errorPropTestEnable
            checkErrorPropagation(truth2nav(x_buff(:,i),simpar), xhat_buff(:,i),...
                delx_buff(:,i), simpar,i);
        end
        %Adjust the Kalman update index
        k_ibc = k_ibc + 1;
        %   For each available measurement
        %       Synthesize the noisy measurement, ztilde
        %       Predict the measurement, ztildehat
        %       Compute the measurement sensitivity matrix, H
        %       If simpar.general.measLinerizationCheckEnable == true
        %           Check measurement linearization
        %       Compute and save the residual
        %       Compute and save the residual covariance
        %       Compute and save the Kalman gain, K
        %       Estimate the error state vector
        %       Update and save the covariance matrix
        %       Correct and save the navigation states
        if simpar.general.process_IBC_enable
            ztilde_ibc_buff(:,k_ibc) = ibc.synthesize_measurement(truth2nav(x_buff(:,i), simpar), simpar);
            ztildehat_ibc_buff(:,k_ibc) = ibc.predict_measurement(xhat_buff(:,i), simpar);
            H_ibc = ibc.compute_H(xhat_buff(:,i), simpar);
            ibc.validate_linearization(x_buff(:,i), simpar);
            res_ibc(:,k_ibc) = ibc.compute_residual(ztilde_ibc_buff(:,k_ibc), ztildehat_ibc_buff(:,k_ibc));
            
            %         resCov_ibc(:,k) = compute_residual_cov();
            %         K_ibc_buff(:,:,k) = compute_Kalman_gain();
            %         del_x = estimate_error_state_vector();
            %         P_buff(:,:,k) = update_covariance();
            %         xhat_buff(:,i) = correctErrors();
        end
    end
    if verbose && mod(i,100) == 0
        fprintf('%0.1f%% complete\n',100 * i/nstep);
    end
end

if verbose
    fprintf('%0.1f%% complete\n',100 * t(i)/t(end));
end

T_execution = toc;
%Package up residuals
navRes.ibc = res_ibc;
% navResCov.ibc = resCov_ibc;
% kalmanGains.ibc = K_ibc_buff;

navResCov.ibc = 0;
kalmanGains.ibc = 0;
%Package up outputs
traj = struct('navState',xhat_buff,...
    'navCov',P_buff,...
    'navRes',navRes,...
    'navResCov',navResCov,...
    'truthState',x_buff,...
    'input', system_input,...
    'time_nav',t,...
    'time_kalman',t_kalman_ibc,...
    'executionTime',T_execution,...
    'continuous_measurements',ytilde_buff,...
    'kalmanGain',kalmanGains,...
    'simpar',simpar,...
    'meas_ibc', ztilde_ibc_buff,...
    'pred_ibc', ztildehat_ibc_buff);
end