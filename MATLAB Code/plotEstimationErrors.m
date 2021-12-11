function h_figs = plotEstimationErrors(traj, simpar)
%plotEstimationErrors plots residuals and estimation errors for a
%single Monte Carlo run.  You need to include residuals and the +/- 3-sigma
%for all measurements that are active during this run.  You should also
%plot the estimation error with it's +/- 3-sigma for each state. I left 
%some example code from a Lunar Lander project so you can see how these
%plots are created.  I also left some example code that converts the errors 
%and covariances to a different coordinate system.
%% Extract true states, nav states, and covariances
navState = traj.navState;
navCov = traj.navCov;
nav_errors = calcErrors( navState, truth2nav(traj.truthState,simpar), simpar );

h_figs = [];
%% PDOA residuals
if simpar.general.process_IBC_enable
    axis_str = {'PDOA'};
    h_figs(end+1) = figure('Name',sprintf('res_pdoa')); %#ok<*AGROW>
    stairs(traj.time_kalman.ibc,traj.navRes.ibc(:)'); hold on
    stairs(traj.time_kalman.ibc,...
        3.*sqrt(squeeze(traj.navResCov_ibc(:))),'r--');
    stairs(traj.time_kalman.ibc,...
        -3.*sqrt(squeeze(traj.navResCov_ibc(:))),'r--');
    xlabel('time$\left[s\right]$','Interpreter','latex');
    ystring = sprintf(' PDOA Residual [rad]');
    ylabel(ystring,'Interpreter','latex')
    grid on;
end
%% GPS residuals
if simpar.general.process_GPS_enable
    axis_str = {'E','N','U'};
    for i=1:3
        h_figs(end+1) = figure('Name',sprintf('res_gps_%d',i)); %#ok<*AGROW>
        stairs(traj.time_kalman.gps,traj.navRes.gps(i,:)'); hold on
        stairs(traj.time_kalman.gps,...
            3.*sqrt(squeeze(traj.navResCov_gps(i,i,:))),'r--');
        stairs(traj.time_kalman.gps,...
            -3.*sqrt(squeeze(traj.navResCov_gps(i,i,:))),'r--');
        xlabel('time$\left[s\right]$','Interpreter','latex');
        ystring = sprintf('%s GPS Residual [m]',axis_str{i});
        ylabel(ystring,'Interpreter','latex')
        grid on;
    end
end
%% Position estimation error
axis_str = {'E','N','U'};
i_axis = 0;
for i=simpar.states.ixfe.pos
    i_axis = i_axis + 1;
    h_figs(end+1) = figure('Name',sprintf('est_err_%d',i)); %#ok<*AGROW>
    stairs(traj.time_nav, nav_errors(i,:)); hold on
    stairs(traj.time_kalman.ibc,traj.del_x_ibc(i,:)); 
    stairs(traj.time_kalman.gps,traj.del_x_gps(i,:)); 
    stairs(traj.time_nav, 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav,-3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time [s]')
    ystring = sprintf('%s Position Est Err [m]',axis_str{i_axis});
    ylabel(ystring)
    legend('actual error','ibc estimated error','gps estimated error','3-sigma')
    grid on;
end
%% Velocity esimation error
axis_str = {'E','N','U'};
i_axis = 0;
for i=simpar.states.ixfe.vel
    i_axis = i_axis + 1;
    h_figs(end+1) = figure('Name',sprintf('est_err_%d',i)); %#ok<*AGROW>
    stairs(traj.time_nav, nav_errors(i,:)); hold on
    stairs(traj.time_kalman.ibc,traj.del_x_ibc(i,:)); 
    stairs(traj.time_kalman.gps,traj.del_x_gps(i,:)); 
    stairs(traj.time_nav, 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav,-3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time [s]')
    ystring = sprintf('%s Velocity Est Err [m/s]',axis_str{i_axis});
    ylabel(ystring)
    legend('actual error','ibc estimated error','gps estimated error','3-sigma')
    grid on;
end
%% Attitude estimation error
axis_str = {'E','N','U'};
i_axis = 0;
for i=simpar.states.ixfe.att
    i_axis = i_axis + 1;
    h_figs(end+1) = figure('Name',sprintf('est_err_%d',i)); %#ok<*AGROW>
    stairs(traj.time_nav, nav_errors(i,:)); hold on
    stairs(traj.time_kalman.ibc,traj.del_x_ibc(i,:)); 
    stairs(traj.time_kalman.gps,traj.del_x_gps(i,:)); 
    stairs(traj.time_nav, 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav, -3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time [s]')
    ystring = sprintf('%s Attitude Est Err [rad]',axis_str{i_axis});
    ylabel(ystring)
    legend('actual error','ibc estimated error','gps estimated error','3-sigma')
    grid on;
end
%% Accelerometer bias estimation error
axis_str = {'X','Y','Z'};
i_axis = 0;
for i=simpar.states.ixfe.abias
    i_axis = i_axis + 1;
    h_figs(end+1) = figure('Name',sprintf('est_err_%d',i)); %#ok<*AGROW>
    stairs(traj.time_nav, nav_errors(i,:)); hold on
    stairs(traj.time_kalman.ibc,traj.del_x_ibc(i,:)); 
    stairs(traj.time_kalman.gps,traj.del_x_gps(i,:)); 
    stairs(traj.time_nav, 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav, -3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time [s]')
    ystring = sprintf('%s Accelerometer Bias Est Err [m/s^2])',axis_str{i_axis});
    ylabel(ystring)
    legend('actual error','ibc estimated error','gps estimated error','3-sigma')
    grid on;
end
%% Gyroscope bias estimation error
axis_str = {'X','Y','Z'};
i_axis = 0;
for i=simpar.states.ixfe.gbias
    i_axis = i_axis + 1;
    h_figs(end+1) = figure('Name',sprintf('est_err_%d',i)); %#ok<*AGROW>
    stairs(traj.time_nav, nav_errors(i,:)); hold on
    stairs(traj.time_kalman.ibc,traj.del_x_ibc(i,:)); 
    stairs(traj.time_kalman.gps,traj.del_x_gps(i,:)); 
    stairs(traj.time_nav, 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav, -3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time [s]')
    ystring = sprintf('%s Gyroscope Bias Est Err (rad/s)',axis_str{i_axis});
    ylabel(ystring)
    legend('actual error','ibc estimated error','gps estimated error','3-sigma')
    grid on;
end
%% Ground Coil Position estimation error
axis_str = {'E','N','U'};
i_axis = 0;
for i=simpar.states.ixfe.cpos
    i_axis = i_axis + 1;
    h_figs(end+1) = figure('Name',sprintf('est_err_%d',i)); %#ok<*AGROW>
    stairs(traj.time_nav, nav_errors(i,:)); hold on
    stairs(traj.time_kalman.ibc,traj.del_x_ibc(i,:)); 
    stairs(traj.time_kalman.gps,traj.del_x_gps(i,:)); 
    stairs(traj.time_nav, 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav, -3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time [s]')
    ystring = sprintf('%s Ground Coil Position Est Err [m]',axis_str{i_axis});
    ylabel(ystring)
    legend('actual error','ibc estimated error','gps estimated error','3-sigma')
    grid on;
end
end