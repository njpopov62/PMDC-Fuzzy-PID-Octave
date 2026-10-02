function [t, y, u_hist, ydot] = simulate_pid(Kp, Ki, Kd, u_sat)
%==========================================================================
% simulate_pid.m
%
% Conventional PID simulation for the PMDC motor.
%
% Direct GNU Octave translation of the original Python:
%
%     def simulate_pid(Kp, Ki, Kd, u_sat=24):
%
% No algorithmic modification is made.
%==========================================================================

    %----------------------------------------------------------------------
    % Default saturation limit
    %----------------------------------------------------------------------

    if nargin < 4
        u_sat = 24;
    end

    %----------------------------------------------------------------------
    % DC MOTOR PARAMETERS
    %----------------------------------------------------------------------

    Ra = 1.0;
    La = 0.5;

    J  = 0.01;
    B  = 0.1;

    Kt = 0.01;
    Ke = 0.01;

    %----------------------------------------------------------------------
    % Simulation parameters
    %----------------------------------------------------------------------

    t_end = 2.0;
    dt = 1e-4;

    N = floor(t_end / dt) + 1;

    t = linspace(0, t_end, N);

    % Unit-step reference
    omega_ref = ones(1, N);

    %----------------------------------------------------------------------
    % PID controller
    %
    % Original Python uses:
    %
    %     tau_d = 5e-4
    %----------------------------------------------------------------------

    pid = PID(Kp, Ki, Kd, dt, -u_sat, u_sat, 5e-4);

    %----------------------------------------------------------------------
    % Initial motor state
    %
    % x(1) = omega
    % x(2) = ia
    %----------------------------------------------------------------------

    x = [0; 0];

    %----------------------------------------------------------------------
    % Output histories
    %----------------------------------------------------------------------

    y = zeros(1, N);
    ydot = zeros(1, N);
    u_hist = zeros(1, N);

    %----------------------------------------------------------------------
    % Simulation loop
    %----------------------------------------------------------------------

    for k = 1:N

        % Current motor states
        omega = x(1);
        ia    = x(2);

        % Motor output
        y(k) = omega;

        % Motor speed derivative
        %
        % Original Python:
        %
        %     ydot[k] = (Kt*ia - B*omega) / J
        %
        ydot(k) = (Kt*ia - B*omega) / J;

        % Control error
        error = omega_ref(k) - omega;

        % PID controller update
        %
        % The measured plant derivative is supplied to the PID,
        % exactly as in the original Python implementation.
        [pid, u] = PID_update(pid, error, ydot(k));

        % Store control signal
        u_hist(k) = u;

        % Update motor state using Forward Euler
        x = plant_step(x, u, dt, ...
                       Ra, La, J, B, Kt, Ke);

    end

end
