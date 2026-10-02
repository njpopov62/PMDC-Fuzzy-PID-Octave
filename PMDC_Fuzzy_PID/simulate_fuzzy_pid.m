function [t, y, u_hist, ydot] = simulate_fuzzy_pid( ...
    Kp, Ki, Kd, ...
    e_scale, de_scale, ...
    dKp_max, dKi_max, dKd_max, ...
    adapt_rate, u_sat)
%==========================================================================
% simulate_fuzzy_pid.m
%
% Fuzzy PID simulation for the PMDC motor.
%
% Direct GNU Octave translation of the original Python:
%
%     def simulate_fuzzy_pid(...):
%
% The algorithm, normalization, fuzzy inference, gain adaptation,
% smoothing, PID update, and motor model are preserved.
%
% No algorithmic modification is made.
%==========================================================================

    %----------------------------------------------------------------------
    % Default parameters
    % These correspond exactly to the original Python function defaults.
    %----------------------------------------------------------------------
    if nargin < 4
        e_scale = 1.2;
    end

    if nargin < 5
        de_scale = 0.6;
    end

    if nargin < 6
        dKp_max = 25;
    end

    if nargin < 7
        dKi_max = 50;
    end

    if nargin < 8
        dKd_max = 4;
    end

    if nargin < 9
        adapt_rate = 0.6;
    end

    if nargin < 10
        u_sat = 24;
    end

    %----------------------------------------------------------------------
    % DC MOTOR PARAMETERS
    % Same values as the original Python code.
    %----------------------------------------------------------------------
    Ra = 1.0;
    La = 0.5;
    J  = 0.01;
    B  = 0.1;
    Kt = 0.01;
    Ke = 0.01;

    %----------------------------------------------------------------------
    % SIMULATION PARAMETERS
    %----------------------------------------------------------------------
    t_end = 2.0;
    dt = 1e-4;

    N = floor(t_end / dt) + 1;

    t = linspace(0, t_end, N);

    omega_ref = ones(1, N);

    %----------------------------------------------------------------------
    % PID controller
    %
    % Original Python:
    %
    %     pid = PID(Kp, Ki, Kd, dt, -u_sat, u_sat, tau_d=5e-4)
    %----------------------------------------------------------------------
    pid = PID(Kp, Ki, Kd, dt, -u_sat, u_sat, 5e-4);

    %----------------------------------------------------------------------
    % Fuzzy tuner
    %----------------------------------------------------------------------
    tuner = FuzzyTuner();

    %----------------------------------------------------------------------
    % State initialization
    %----------------------------------------------------------------------
    x = [0; 0];

    y = zeros(1, N);
    ydot = zeros(1, N);
    u_hist = zeros(1, N);

    %----------------------------------------------------------------------
    % Base controller gains
    %----------------------------------------------------------------------
    Kp_c = Kp;
    Ki_c = Ki;
    Kd_c = Kd;

    % Gain smoothing factor
    alpha = 0.02;

    %----------------------------------------------------------------------
    % Main simulation loop
    %----------------------------------------------------------------------
    for k = 1:N

        % Current motor state
        omega = x(1);
        ia    = x(2);

        % Plant output
        y(k) = omega;

        % Plant output derivative
        ydot(k) = (Kt*ia - B*omega) / J;

        % Control error
        error = omega_ref(k) - omega;

        %------------------------------------------------------------------
        % Normalize error
        % Original Python:
        %
        % e_norm = np.clip(error/e_scale, -1, 1)
        %------------------------------------------------------------------
        e_norm = error / e_scale;

        if e_norm > 1
            e_norm = 1;
        elseif e_norm < -1
            e_norm = -1;
        end

        %------------------------------------------------------------------
        % Normalize error derivative
        %
        % Original Python:
        %
        % de_norm = np.clip((-ydot)/de_scale, -1, 1)
        %------------------------------------------------------------------
        de_norm = (-ydot(k)) / de_scale;

        if de_norm > 1
            de_norm = 1;
        elseif de_norm < -1
            de_norm = -1;
        end

        %------------------------------------------------------------------
        % Fuzzy inference
        %------------------------------------------------------------------
        [dKp_norm, dKi_norm, dKd_norm] = ...
            infer(tuner, e_norm, de_norm);

        %------------------------------------------------------------------
        % Scale fuzzy gain corrections
        %------------------------------------------------------------------
        dKp = adapt_rate * dKp_max * dKp_norm;
        dKi = adapt_rate * dKi_max * dKi_norm;
        dKd = adapt_rate * dKd_max * dKd_norm;

        %------------------------------------------------------------------
        % Smooth adaptive gains
        %
        % Original Python:
        %
        % alpha = 0.02
        %
        % Kp_c = (1-alpha)*Kp_c + alpha*(Kp+dKp)
        % Ki_c = (1-alpha)*Ki_c + alpha*(Ki+dKi)
        % Kd_c = (1-alpha)*Kd_c + alpha*(Kd+dKd)
        %------------------------------------------------------------------
        Kp_c = (1 - alpha)*Kp_c + alpha*(Kp + dKp);
        Ki_c = (1 - alpha)*Ki_c + alpha*(Ki + dKi);
        Kd_c = (1 - alpha)*Kd_c + alpha*(Kd + dKd);

        %------------------------------------------------------------------
        % Temporarily apply adaptive gains to PID controller
        %
        % This corresponds to the original Python behavior:
        %
        %     old = (pid.Kp, pid.Ki, pid.Kd)
        %     pid.Kp, pid.Ki, pid.Kd = Kp_c, Ki_c, Kd_c
        %     u = pid.update(...)
        %     pid.Kp, pid.Ki, pid.Kd = old
        %------------------------------------------------------------------
        old_Kp = pid.Kp;
        old_Ki = pid.Ki;
        old_Kd = pid.Kd;

        pid.Kp = Kp_c;
        pid.Ki = Ki_c;
        pid.Kd = Kd_c;

        [pid, u] = PID_update(pid, error, ydot(k));

        pid.Kp = old_Kp;
        pid.Ki = old_Ki;
        pid.Kd = old_Kd;

        % Store control signal
        u_hist(k) = u;

        %------------------------------------------------------------------
        % PMDC motor plant update
        %------------------------------------------------------------------
        x = plant_step(x, u, dt, ...
                       Ra, La, J, B, Kt, Ke);

    end

end
