function h_figs = plotNavPropErrors(traj)
% In this function, you should plot everything you want to see for a
% single simulation when there are no errors present.  I like to see things
% like the 3D trajectory, altitude, angular rates, etc.  Two sets of plots
% that are required are the estimation errors for every state and the
% residuals for any measurements.  I left some of my code for a lunar
% lander application, so you can see how I do it.  Feel free to use or
% remove whatever applies to your problem.
%% Prelims
 h_figs = [];
simpar = traj.simpar;
%% Plot Steering Angle
h_figs(end+1) = figure;
st_angle = traj.truthState(simpar.states.ix.st_angle,:);
stairs(traj.time_nav, st_angle(1,:)*180/pi);
xlabel('time [s]');
ylabel('Phi [deg]');
title('Steering Angle');

%% Plot Heading Angle
h_figs(end+1) = figure;
true_q = traj.truthState(simpar.states.ix.att,:);
true_dcm = q2tmat(true_q);
true_psi = squeeze((180/pi)*atan(true_dcm(1,2,:)./true_dcm(1,1,:)));
stairs(traj.time_nav, true_psi);
hold on;
nav_q = traj.navState(simpar.states.ixf.att,:);
nav_dcm = q2tmat(nav_q);
nav_psi = squeeze((180/pi)*atan(nav_dcm(1,2,:)./nav_dcm(1,1,:)));
stairs(traj.time_nav, nav_psi);
title('True vs Estimated Heading Angle');
xlabel('time [s]');
ylabel('Heading Angle [deg]');
legend('True','Estimated')
hold off;
grid on;


%% Plot Measured Angular Rate
h_figs(end+1) = figure;
omega_z = traj.continuous_measurements(6,:);
stairs(traj.time_nav, omega_z);
title('Measured Body-Frame Angular Rate');
xlabel('time [s]');
ylabel('Angular Rate [rad/s]');
grid on;

%% Plot Position
h_figs(end+1) = figure;
true_position = traj.truthState(simpar.states.ix.pos,:);
stairs(true_position(1,:), true_position(2,:));
title('True vs Estimated Position');
hold on;
nav_position = traj.navState(simpar.states.ixf.pos,:);
stairs(nav_position(1,:), nav_position(2,:));
legend('True','Estimated')
xlabel('East');
ylabel('North');
grid on;
hold off;

%% Plot East-Velocity
h_figs(end+1) = figure;
true_velocity = traj.truthState(simpar.states.ix.vel,:);
stairs(traj.time_nav, true_velocity(1,:));
hold on;
nav_velocity = traj.navState(simpar.states.ixf.vel,:);
stairs(traj.time_nav, nav_velocity(1,:));
title('True vs Estimated East-Velocity');
legend('True','Estimated')
xlabel('time [s]');
ylabel('Velocity [m/s]');
grid on;
hold off;

%% Plot North-Velocity
h_figs(end+1) = figure;
%true_velocity = traj.truthState(simpar.states.ix.vel,:);
stairs(traj.time_nav, true_velocity(2,:));
hold on;
%nav_velocity = traj.navState(simpar.states.ixf.vel,:);
stairs(traj.time_nav, nav_velocity(2,:));
title('True vs Estimated North-Velocity');
legend('True','Estimated')
xlabel('time [s]');
ylabel('Velocity [m/s]');
grid on;
hold off;

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


%% Plot True Inertial-Frame East-Acceleration
h_figs(end+1) = figure;
% i_east_accel = body_accel./cos(true_psi*pi/180);
temp = size(traj.continuous_measurements);
nstep = temp(2);
i_accel_quat = qmult(qConjugate(true_q), qmult([zeros(1,nstep); body_accel], true_q));
i_accel = i_accel_quat([2 3 4], :);
i_accel_east = i_accel(1,:);
i_accel_north = i_accel(2,:);
i_accel_up = i_accel(3,:);
hold on;
stairs(traj.time_nav, i_accel_east);
stairs(traj.time_nav, i_accel_north);
stairs(traj.time_nav, i_accel_up);
title('True Inertial-Frame Acceleration');
xlabel('time [s]');
ylabel('Acceleration [m/s^2]');
legend('East','North','Up')
grid on;
hold off;
spreadfigures();
%% Calculate estimation errors
dele = calcErrors(traj.navState, traj.truthState, simpar);

%% Plot position estimation error
h_figs(end+1) = figure;
stairs(traj.time_nav, dele(simpar.states.ixfe.pos,:)');
title('Position Error');
xlabel('time(s)');
ylabel('m');
legend('$x_i$','$y_i$','$z_i$')
grid on;
%% Plot velocity error
h_figs(end+1) = figure;
stairs(traj.time_nav, dele(simpar.states.ixfe.vel,:)');
title('Velocity Error');
xlabel('time(s)');
ylabel('m/s');
legend('x_i','y_i','z_i')
grid on;
%% Add the remaining estimation error plots
end