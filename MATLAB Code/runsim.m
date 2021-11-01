function [ traj ] = runsim( simpar, verbose, seed)
rng(seed);
%RUNSIM Runs a single trajectory given the parameters in simparams
tic;
%% Prelims
%Derive the number of steps in the simulation and the time
nstep = ceil(simpar.general.tsim/simpar.general.dt + 1);
nstep_aid = ceil(simpar.general.tsim/simpar.general.dt_kalmanUpdate);
t = (0:nstep-1)'*simpar.general.dt;
t_kalman = (0:nstep_aid)'.*simpar.general.dt_kalmanUpdate;
nstep_aid = length(t_kalman);
%If you are computing the nominal star tracker or other sensor orientations
%below is an example of one way to do this
% % qz = rotv2quat(simpar.general.thz_st,[0,0,1]');
% % qy = rotv2quat(simpar.general.thy_st,[0,1,0]');
% % qx = rotv2quat(simpar.general.thx_st,[1,0,0]');
% % simpar.general.q_b2st_nominal = qmult(qx,qmult(qy,qz));
% % qz = rotv2quat(simpar.general.thz_c,[0,0,1]');
% % qy = rotv2quat(simpar.general.thy_c,[0,1,0]');
% % qx = rotv2quat(simpar.general.thx_c,[1,0,0]');
% % simpar.general.q_b2c_nominal = qmult(qx,qmult(qy,qz));
%% Pre-allocate buffers for saving data
% Truth, navigation, and error state buffers
x_buff          = zeros(simpar.states.nx,nstep);
xhat_buff       = zeros(simpar.states.nxf,nstep);
delx_buff       = zeros(simpar.states.nxfe,nstep);
% Navigation covariance buffer
P_buff       = zeros(simpar.states.nxfe,simpar.states.nxfe,nstep);
% Continuous measurement buffer
ytilde_buff     = zeros(simpar.general.n_inertialMeas,nstep);
% Residual buffers (star tracker is included as an example)
%TODO: Replace the example buffers with ibc buffers
res_ibc     = zeros(1,nstep_aid);
resCov_ibc  = zeros(3,3,nstep_aid);
K_ibc_buff  = zeros(simpar.states.nxfe,3,nstep_aid);
ztilde_ibc_buff = zeros(1,nstep_aid);
ztildehat_ibc_buff = zeros(1,nstep_aid);
%% Initialize the navigation covariance matrix
% P_buff(:,:,1) = initialize_covariance();
%% Initialize the truth state vector
x_buff(:,1) = initialize_truth_state(simpar);
%% Initialize the navigation state vector
xhat_buff(:,1) = initialize_nav_state(simpar, x_buff(:,1));
%% Miscellaneous calcs
% Synthesize continuous sensor data at t_n-1

% Vehicle accelerates constantly with zero steering angle headed North for 
% accel_time. After accel_time, vehicle continues at w/constant acceleration 
% and w/sinusoidal steering rate.
time_scalar = 1/simpar.general.dt;
accel_time = 5; % [s]
accel_1_value = 0.5; % [m/s^2]
accel_2_value = 0; % [m/s^2]
steering_rate = 0.5*pi/180; %[deg/s] to [rad/s]
a_y = [accel_1_value*ones(accel_time*time_scalar,1); ...
    accel_2_value*ones(nstep-accel_time*time_scalar,1)];
xi = [zeros(accel_time*time_scalar,1); ...
    steering_rate*cos(1*t(1:nstep-(accel_time*time_scalar)))];
% xi = [steering_rate*ones(accel_time*time_scalar,1);...
%        zeros(nstep-(accel_time*time_scalar),1)];

ytilde_buff(:,1) = contMeas(x_buff(:,1), a_y(1), simpar);
%Initialize the measurement counter
k = 1;
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
    xhat_buff(:,1) = injectErrors(x_buff(:,1), delx_buff(:,1), simpar);
end
%% Loop over each time step in the simulation
for i=2:nstep
    % Propagate truth states to t_n
    %   Realize a sample of process noise (don't forget to scale Q by 1/dt!)
    %   Define any inputs to the truth state DE
    %   Perform one step of RK4 integration
    input_truth.u = [a_y(i); xi(i)];
    %TODO: Synthesize noise
    w_a = [0; 0; 0;];
    w_g = [0; 0; 0;];
    
    input_truth.w = [w_a; w_g];
    input_truth.simpar = simpar;
    
    % Propagate truth state forward one dt
    x_buff(:,i) = rk4('truthState_de',x_buff(:,i-1), input_truth,...
        simpar.general.dt);
    % xnew = rk4(diffeq,xold,input,dt)
    
    % Synthesize continuous sensor data at t_n
    ytilde_buff(:,i) = contMeas(x_buff(:,i), a_y(i), simpar);
    
    % Propagate navigation states to t_n using sensor data from t_n-1
    %   Assign inputs to the navigation state DE
    %   Perform one step of RK4 integration
    input_nav.a_tilde = ytilde_buff([1 2 3],i);
    input_nav.omega_tilde = ytilde_buff([4 5 6],i);
    input_nav.simpar = simpar;
    xhat_buff(:,i) = rk4('navState_de', xhat_buff(:,i-1), input_nav, ...
        simpar.general.dt);
    % Propagate the covariance to t_n
%     input_cov.ytilde = [];
%     input_cov.simpar = simpar;
%     P_buff(:,:,i) = rk4('navCov_de', P_buff(:,:,i-1), input_cov, ...
%         simpar.general.dt);
    % Propagate the error state from tn-1 to tn if errorPropTestEnable == 1
    if simpar.general.errorPropTestEnable
        input_delx.xhat = xhat_buff(:,i-1);
        input_delx.ytilde = ytilde_buff(:,i);
        input_delx.simpar = simpar;
        delx_buff(:,i) = rk4('errorState_de', delx_buff(:,i-1), ...
            input_delx, simpar.general.dt);
        % rk4(diffeq,xold,input,dt)
    end
    
    % If discrete measurements are available, perform a Kalman update
    if abs(t(i)-t_kalman(k+1)) < simpar.general.dt*0.01
        %   Check error state propagation if simpar.general.errorPropTestEnable = true
        if simpar.general.errorPropTestEnable
            checkErrorPropagation(x_buff(:,i), xhat_buff(:,i),...
                delx_buff(:,i), simpar);
        end
        %Adjust the Kalman update index
        k = k + 1;
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
        %TODO: Create buffers for the ibc measurement
        ztilde_ibc_buff(:,k) = ibc.synthesize_measurement(truth2nav(x_buff(:,i), simpar), simpar);
        ztildehat_ibc_buff(:,k) = ibc.predict_measurement(xhat_buff(:,i), simpar);
        
%         H_ibc = ibc.compute_H();
%         ibc.validate_linearization();
         res_ibc(:,k) = ibc.compute_residual(ztilde_ibc_buff(:,k), ztildehat_ibc_buff(:,k));
         
%         resCov_ibc(:,k) = compute_residual_cov();
%         K_ibc_buff(:,:,k) = compute_Kalman_gain();
%         del_x = estimate_error_state_vector();
%         P_buff(:,:,k) = update_covariance();
%         xhat_buff(:,i) = correctErrors();
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
    'time_nav',t,...
    'time_kalman',t_kalman,...
    'executionTime',T_execution,...
    'continuous_measurements',ytilde_buff,...
    'kalmanGain',kalmanGains,...
    'simpar',simpar,...
    'meas_ibc', ztilde_ibc_buff,...
    'pred_ibc', ztildehat_ibc_buff);
end