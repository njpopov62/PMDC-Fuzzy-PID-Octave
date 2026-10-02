function [pid, u_sat] = PID_update(pid, error, y_dot)
%==========================================================================
% PID_update.m
%
% Perform one PID controller update.
%
% Direct GNU Octave translation of the original Python:
%
%     def update(self, error, y_dot=None):
%
% Inputs:
%     pid   - PID controller structure
%     error - current control error
%     y_dot - plant output derivative (optional)
%
% Outputs:
%     pid   - updated PID controller structure
%     u_sat - saturated control output
%
% No algorithmic modification is made.
%==========================================================================

    %----------------------------------------------------------------------
    % Proportional term
    %
    %     up = Kp * error
    %----------------------------------------------------------------------

    up = pid.Kp * error;

    %----------------------------------------------------------------------
    % Integral term
    %
    %     integral += error * dt
    %     ui = Ki * integral
    %----------------------------------------------------------------------

    pid.integral = pid.integral + error * pid.dt;

    ui = pid.Ki * pid.integral;

    %----------------------------------------------------------------------
    % Derivative term
    %
    % Original Python:
    %
    %     if y_dot is None:
    %         de = (error - prev_error) / dt
    %     else:
    %         de = -y_dot
    %
    % In the motor simulation y_dot is supplied, so normally:
    %
    %     de = -y_dot
    %----------------------------------------------------------------------

    if nargin < 3 || isempty(y_dot)
        de = (error - pid.prev_error) / pid.dt;
    else
        de = -y_dot;
    end

    %----------------------------------------------------------------------
    % First-order derivative filter
    %
    %     d_state += dt * (de - d_state) / max(tau_d, 1e-6)
    %----------------------------------------------------------------------

    tau = max(pid.tau_d, 1e-6);

    pid.d_state = pid.d_state + ...
                  pid.dt * (de - pid.d_state) / tau;

    % Derivative contribution
    ud = pid.Kd * pid.d_state;

    %----------------------------------------------------------------------
    % PID output before saturation
    %----------------------------------------------------------------------

    u = up + ui + ud;

    %----------------------------------------------------------------------
    % Output saturation
    %
    % Equivalent to:
    %
    %     u_sat = np.clip(u, u_min, u_max)
    %----------------------------------------------------------------------

    u_sat = min(max(u, pid.u_min), pid.u_max);

    %----------------------------------------------------------------------
    % Anti-windup
    %
    % The original Python code contains:
    %
    %     if self.Ki > 0:
    %         self.integral += (u_sat - u) * 0.0
    %
    % This is intentionally preserved exactly.
    % It therefore has NO effect on the integral state.
    %----------------------------------------------------------------------

    if pid.Ki > 0
        pid.integral = pid.integral + (u_sat - u) * 0.0;
    end

    %----------------------------------------------------------------------
    % Save current error for the next update
    %----------------------------------------------------------------------

    pid.prev_error = error;

end
