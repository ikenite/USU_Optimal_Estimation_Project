function h_figs = plotNavPropErrors(traj)
% In this function, you should plot everything you want to see for a
% single simulation when there are no errors present.  I like to see things
% like the 3D trajectory, altitude, angular rates, etc.  Two sets of plots
% that are required are the estimation errors for every state and the
% residuals for any measurements.  I left some of my code for a lunar
% lander application, so you can see how I do it.  Feel free to use or
% remove whatever applies to your problem.
%% Prelims
states = 0;
measurements = 0;
estimationErrors = 1;
residuals = 1;

 h_figs = [];
simpar = traj.simpar;

%% State Plots
if states == true
    %% Plot Vehicle Position
    h_figs(end+1) = figure;
    hold on;
    true_position_E = traj.truthState(simpar.states.ix.pos_E,:);
    true_position_N = traj.truthState(simpar.states.ix.pos_N,:);
    stairs(true_position_E(:), true_position_N(:));
    nav_position_E = traj.navState(simpar.states.ixf.pos(1),:);
    nav_position_N = traj.navState(simpar.states.ixf.pos(2),:);
    stairs(nav_position_E(:), nav_position_N(:));
    title('True vs Estimated Vehicle Position');
    legend('True','Estimated')
    xlabel('East');
    ylabel('North');
    grid on;
    hold off;

    %% Plot Velocity
    h_figs(end+1) = figure;
    true_velocity = traj.truthState(simpar.states.ix.vel_yb,:);
    stairs(traj.time_nav, true_velocity(1,:));
    hold on;
    nav_velocity = vecnorm(traj.navState(simpar.states.ixf.vel,:));
    stairs(traj.time_nav, nav_velocity(1,:));
    title('True vs Estimated Velocity');
    legend('True','Estimated')
    xlabel('time [s]');
    ylabel('Velocity [m/s]');
    grid on;
    hold off;
    
    %% Plot Heading Angle
    h_figs(end+1) = figure;
    true_psi = traj.truthState(simpar.states.ix.head_angle,:);
    stairs(traj.time_nav, true_psi*180/pi);
    hold on;
    nav_q = traj.navState(simpar.states.ixf.att,:);
    nav_dcm = q2tmat(nav_q);
    nav_psi = squeeze((180/pi)*atan2(nav_dcm(1,2,:),nav_dcm(1,1,:)));
    stairs(traj.time_nav, nav_psi);
    title('True vs Estimated Heading Angle');
    xlabel('time [s]');
    ylabel('Heading Angle [deg]');
    legend('True','Estimated')
    hold off;
    grid on;
    
    %% Plot Steering Angle
    h_figs(end+1) = figure;
    st_angle = traj.truthState(simpar.states.ix.st_angle,:);
    stairs(traj.time_nav, st_angle(1,:)*180/pi);
    xlabel('time [s]');
    ylabel('Phi [deg]');
    title('Steering Angle');   
    
    %% Plot Accelerometer Bias
    h_figs(end+1) = figure;
    true_accel_bias = traj.truthState(simpar.states.ix.abias,:);
    true_accel_bias_x = true_accel_bias(1,:);
    true_accel_bias_y = true_accel_bias(2,:);
    true_accel_bias_z = true_accel_bias(3,:);
    est_accel_bias = traj.navState(simpar.states.ixf.abias,:);
    est_accel_bias_x = est_accel_bias(1,:);
    est_accel_bias_y = est_accel_bias(2,:);
    est_accel_bias_z = est_accel_bias(3,:);
    hold on;
    stairs(traj.time_nav, true_accel_bias_x);
    stairs(traj.time_nav, true_accel_bias_y);
    stairs(traj.time_nav, true_accel_bias_z);
    stairs(traj.time_nav, est_accel_bias_x);
    stairs(traj.time_nav, est_accel_bias_y);
    stairs(traj.time_nav, est_accel_bias_z);
    hold off;
    title('True vs Estimated Accelerometer Biases');
    xlabel('time [s]');
    ylabel('Bias [m/s^2]');
    legend('True x','True y','True z', ...
        'Estimated x','Estimated y','Estimated z');
    grid on;
    
    %% Plot Gyroscope Bias
    h_figs(end+1) = figure;
    true_gyro_bias = traj.truthState(simpar.states.ix.gbias,:);
    true_gyro_bias_x = true_gyro_bias(1,:);
    true_gyro_bias_y = true_gyro_bias(2,:);
    true_gyro_bias_z = true_gyro_bias(3,:);
    est_gyro_bias = traj.navState(simpar.states.ixf.gbias,:);
    est_gyro_bias_x = est_gyro_bias(1,:);
    est_gyro_bias_y = est_gyro_bias(2,:);
    est_gyro_bias_z = est_gyro_bias(3,:);
    hold on;
    stairs(traj.time_nav, true_gyro_bias_x);
    stairs(traj.time_nav, true_gyro_bias_y);
    stairs(traj.time_nav, true_gyro_bias_z);
    stairs(traj.time_nav, est_gyro_bias_x);
    stairs(traj.time_nav, est_gyro_bias_y);
    stairs(traj.time_nav, est_gyro_bias_z);
    hold off;
    title('True vs Estimated Gyroscope Biases');
    xlabel('time [s]');
    ylabel('Bias [rad/s]');
    legend('True x','True y','True z', ...
        'Estimated x','Estimated y','Estimated z');
    grid on;
end

%% Measurement Plots
if measurements == true
    %% Plot Measured Body-Frame Accelerations
    h_figs(end+1) = figure;
    body_accel = (traj.continuous_measurements([1 2 3],:));
    body_accel_x = body_accel(1,:);
    body_accel_y = body_accel(2,:);
    body_accel_z = body_accel(3,:);
    hold on;
    stairs(traj.time_nav, body_accel_x);
    stairs(traj.time_nav, body_accel_y);
    stairs(traj.time_nav, body_accel_z);
    hold off;
    title('Measured Body-Frame Accelerations');
    xlabel('time [s]');
    ylabel('Acceleration [m/s^2]');
    legend('x','y','z');
    grid on;

    %% Plot Measured Angular Rate
    h_figs(end+1) = figure;
    omega_z = traj.continuous_measurements(6,:);
    stairs(traj.time_nav, omega_z);
    title('Measured Body-Frame Angular Rate');
    xlabel('time [s]');
    ylabel('Angular Rate [rad/s]');
    grid on;

end

%% Estimation Error Plots
if estimationErrors == true
    %% Calculate estimation errors
    dele = calcErrors(traj.navState, truth2nav(traj.truthState, simpar), simpar);

    %% Plot vehicle position estimation error
    h_figs(end+1) = figure;
    stairs(traj.time_nav, dele(simpar.states.ixfe.pos,:)');
    title('Vehicle Position Error');
    xlabel('time(s)');
    ylabel('m');
    legend('E','N','U')
    grid on;
    %% Plot velocity estimation error
    h_figs(end+1) = figure;
    stairs(traj.time_nav, dele(simpar.states.ixfe.vel,:)');
    title('Velocity Error');
    xlabel('time(s)');
    ylabel('m/s');
    legend('E','N','U')
    grid on;
    %% Plot attitude estimation error
    h_figs(end+1) = figure;
    stairs(traj.time_nav, dele(simpar.states.ixfe.att,:)');
    title('Attitude Error');
    xlabel('time(s)');
    ylabel('(unitless)');
    legend('q2','q3', 'q4')
    grid on;
    %% Plot accelerometer bias estimation error
    h_figs(end+1) = figure;
    stairs(traj.time_nav, dele(simpar.states.ixfe.abias,:)');
    title('Accelerometer Bias Error');
    xlabel('time(s)');
    ylabel('m/s^2');
    legend('x','y','z')
    grid on;
    %% Plot gyro bias estimation error
    h_figs(end+1) = figure;
    stairs(traj.time_nav, dele(simpar.states.ixfe.abias,:)');
    title('Gyroscope Bias Error');
    xlabel('time(s)');
    ylabel('rad/s');
    legend('x','y','z')
    grid on;
    %% Plot ground coil position estimation error
    h_figs(end+1) = figure;
    stairs(traj.time_nav, dele(simpar.states.ixfe.cpos,:)');
    title('Ground Coil Position Error');
    xlabel('time(s)');
    ylabel('m');
    legend('E','N','U')
    grid on;
end
%% Measurement Residual Plots
if residuals == true
   %% Plot Accelerometer Residual  
    h_figs(end+1) = figure;
    stairs(traj.time_kalman, traj.navRes.ibc');
    title('Phase Difference Measurement Residuals');
    xlabel('time(s)');
    ylabel('(deg)');
    grid on;
   %% Plot Gyroscope Residual  
end
spreadfigures();
end