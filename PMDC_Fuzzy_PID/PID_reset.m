function pid = PID_reset(pid)
%==========================================================================
% PID_reset.m
%
% Reset the internal states of the PID controller.
%
% Direct GNU Octave translation of the original Python:
%
%     def reset(self):
%         self.integral = 0.0
%         self.prev_error = 0.0
%         self.d_state = 0.0
%
% The PID gains and configuration parameters are NOT changed.
%
% No algorithmic modification is made.
%==========================================================================

    % Reset integral state
    pid.integral = 0.0;

    % Reset previous error
    pid.prev_error = 0.0;

    % Reset filtered derivative state
    pid.d_state = 0.0;

end
