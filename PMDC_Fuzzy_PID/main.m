%==========================================================================
% main.m
%
% Fuzzy PID vs Conventional PID for PMDC Motor
%
% Direct GNU Octave implementation of the original Python program.
%
% The supporting modules are:
%
%   plant_step.m
%   performance_metrics.m
%   PID.m
%   PID_update.m
%   PID_reset.m
%   tri_mf.m
%   fuzzify.m
%   FuzzyTuner.m
%   infer.m
%   simulate_pid.m
%   simulate_fuzzy_pid.m
%
% No algorithmic modification is made.
%==========================================================================

clear;
close all;
clc;

%--------------------------------------------------------------------------
% BASE PID GAINS
%--------------------------------------------------------------------------
Kp0 = 100;
Ki0 = 200;
Kd0 = 10;

%--------------------------------------------------------------------------
% FUZZY PID PARAMETERS
% These are the parameters used by the original Python main program.
%--------------------------------------------------------------------------
e_scale    = 1.2;
de_scale   = 0.7;

dKp_max    = 28;
dKi_max    = 60;
dKd_max    = 4;

adapt_rate = 0.55;

u_sat = 24;

%--------------------------------------------------------------------------
% RUN CONVENTIONAL PID
%--------------------------------------------------------------------------
printf('\n');
printf('===============================================\n');
printf(' Conventional PID simulation\n');
printf('===============================================\n');

[t, y_pid, u_pid, ydot_pid] = ...
    simulate_pid(Kp0, Ki0, Kd0, u_sat);

%--------------------------------------------------------------------------
% RUN FUZZY PID
%--------------------------------------------------------------------------
printf('\n');
printf('===============================================\n');
printf(' Fuzzy PID simulation\n');
printf('===============================================\n');

[t_fuzzy, y_fuzzy, u_fuzzy, ydot_fuzzy] = ...
    simulate_fuzzy_pid( ...
        Kp0, Ki0, Kd0, ...
        e_scale, de_scale, ...
        dKp_max, dKi_max, dKd_max, ...
        adapt_rate, u_sat);

%--------------------------------------------------------------------------
% PERFORMANCE METRICS
%
% The original Python program explicitly evaluates the responses against
% y_final = 1.0.
%--------------------------------------------------------------------------
y_final = 1.0;

metrics_pid = performance_metrics(t, y_pid, y_final);

metrics_fuzzy = performance_metrics( ...
    t_fuzzy, y_fuzzy, y_final);

%--------------------------------------------------------------------------
% PRINT RESULTS
%--------------------------------------------------------------------------
printf('\n');
printf('===============================================\n');
printf(' PERFORMANCE RESULTS\n');
printf('===============================================\n');

printf('\nConventional PID:\n');
printf('  Rise time      = %.6f s\n', ...
       metrics_pid.rise_time);
printf('  Overshoot      = %.6f %%\n', ...
       metrics_pid.overshoot);
printf('  Settling time  = %.6f s\n', ...
       metrics_pid.settling_time);
printf('  ISE            = %.6f\n', ...
       metrics_pid.ISE);
printf('  ITAE           = %.6f\n', ...
       metrics_pid.ITAE);

printf('\nFuzzy PID:\n');
printf('  Rise time      = %.6f s\n', ...
       metrics_fuzzy.rise_time);
printf('  Overshoot      = %.6f %%\n', ...
       metrics_fuzzy.overshoot);
printf('  Settling time  = %.6f s\n', ...
       metrics_fuzzy.settling_time);
printf('  ISE            = %.6f\n', ...
       metrics_fuzzy.ISE);
printf('  ITAE           = %.6f\n', ...
       metrics_fuzzy.ITAE);

%--------------------------------------------------------------------------
% ADDITIONAL FINAL VALUES
%--------------------------------------------------------------------------
printf('\n');
printf('===============================================\n');
printf(' FINAL SIMULATION VALUES\n');
printf('===============================================\n');

printf('\nConventional PID:\n');
printf('  Final speed     = %.6f\n', y_pid(end));
printf('  Maximum speed   = %.6f\n', max(y_pid));
printf('  Maximum control = %.6f V\n', max(u_pid));
printf('  Minimum control = %.6f V\n', min(u_pid));

printf('\nFuzzy PID:\n');
printf('  Final speed     = %.6f\n', y_fuzzy(end));
printf('  Maximum speed   = %.6f\n', max(y_fuzzy));
printf('  Maximum control = %.6f V\n', max(u_fuzzy));
printf('  Minimum control = %.6f V\n', min(u_fuzzy));

%--------------------------------------------------------------------------
% FIGURE 1
%
% Full step response.
%--------------------------------------------------------------------------
figure(1);

plot(t, y_pid, '--', 'LineWidth', 1.0);
hold on;

plot(t_fuzzy, y_fuzzy, 'LineWidth', 1.8);

plot(t, ones(size(t)), ':', 'LineWidth', 1.0);

grid on;

xlabel('Time (s)');
ylabel('Angular speed');

title('Fuzzy PID vs Conventional PID - PMDC Motor');

legend('Conventional PID', ...
       'Fuzzy PID', ...
       'Reference', ...
       'Location', 'best');

xlim([0 t(end)]);

print('fuzzy_pid_step_response.png', '-dpng', '-r150');

%--------------------------------------------------------------------------
% FIGURE 2
%
% Zoomed response, 0 to 0.35 seconds.
%--------------------------------------------------------------------------
figure(2);

plot(t, y_pid, '--', 'LineWidth', 1.0);
hold on;

plot(t_fuzzy, y_fuzzy, 'LineWidth', 1.8);

plot(t, ones(size(t)), ':', 'LineWidth', 1.0);

grid on;

xlabel('Time (s)');
ylabel('Angular speed');

title('Fuzzy PID vs Conventional PID - Zoomed');

legend('Conventional PID', ...
       'Fuzzy PID', ...
       'Reference', ...
       'Location', 'best');

xlim([0 0.35]);

print('fuzzy_pid_zoomed.png', '-dpng', '-r150');

%--------------------------------------------------------------------------
% FINISHED
%--------------------------------------------------------------------------
printf('\n');
printf('===============================================\n');
printf(' Simulation completed.\n');
printf('===============================================\n');

printf('\nGenerated files:\n');
printf('  fuzzy_pid_step_response.png\n');
printf('  fuzzy_pid_zoomed.png\n');
printf('\n');
