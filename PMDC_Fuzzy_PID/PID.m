function pid = PID(Kp, Ki, Kd, dt, u_min, u_max, tau_d)
%==========================================================================
% PID.m
%
% PID controller initialization.
%
% Direct GNU Octave equivalent of the original Python:
%
%     class PID:
%         def __init__(self, Kp, Ki, Kd, dt,
%                      u_min=-24, u_max=24, tau_d=1e-3):
%
% The controller states are:
%
%     integral   - integral of the error
%     prev_error - previous error
%     d_state    - filtered derivative state
%
% No algorithmic modification is made.
%==========================================================================

    %----------------------------------------------------------------------
    % Default parameters
    %----------------------------------------------------------------------

    if nargin < 5
        u_min = -24;
    end

    if nargin < 6
        u_max = 24;
    end

    if nargin < 7
        tau_d = 1e-3;
    end

    %----------------------------------------------------------------------
    % PID gains and parameters
    %----------------------------------------------------------------------

    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;

    pid.dt = dt;

    pid.u_min = u_min;
    pid.u_max = u_max;

    pid.tau_d = tau_d;

    %----------------------------------------------------------------------
    % Controller state initialization
    %
    % Equivalent to Python:
    %
    %     self.integral = 0.0
    %     self.prev_error = 0.0
    %     self.d_state = 0.0
    %----------------------------------------------------------------------

    pid.integral = 0.0;
    pid.prev_error = 0.0;
    pid.d_state = 0.0;

end
