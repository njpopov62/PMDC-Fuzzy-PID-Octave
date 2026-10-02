function metrics = performance_metrics(t, y, y_final)
%==========================================================================
% performance_metrics.m
%
% Performance metrics for the PMDC motor step response.
%
% Direct GNU Octave translation of the original Python:
%
%     def performance_metrics(t, y, y_final=None):
%
% Metrics:
%     Rise time       : 10% to 90%
%     Overshoot       : percentage
%     Settling time   : 2% criterion
%     ISE             : Integral of Squared Error
%     ITAE            : Integral of Time-weighted Absolute Error
%
% No algorithmic modification is made.
%==========================================================================

    % If y_final is not supplied, use the final value of y.
    if nargin < 3
        y_final = y(end);
    end

    %----------------------------------------------------------------------
    % Rise time: 10% to 90%
    %----------------------------------------------------------------------

    % Find first sample where y >= 10% of final value
    idx10 = find(y >= 0.1*y_final, 1, 'first');

    % Find first sample where y >= 90% of final value
    idx90 = find(y >= 0.9*y_final, 1, 'first');

    if isempty(idx10) || isempty(idx90)
        rise_time = NaN;
    else
        rise_time = t(idx90) - t(idx10);
    end

    %----------------------------------------------------------------------
    % Overshoot
    %----------------------------------------------------------------------

    peak = max(y);

    overshoot = max( ...
        ((peak - y_final) / (y_final + 1e-12)) * 100, ...
        0);

    %----------------------------------------------------------------------
    % Settling time: 2% criterion
    %
    % First find the earliest point after which ALL remaining samples
    % remain inside:
    %
    %     0.98*y_final <= y <= 1.02*y_final
    %----------------------------------------------------------------------

    inside = (y >= 0.98*y_final) & ...
             (y <= 1.02*y_final);

    settling_idx = [];

    % Direct equivalent of:
    %
    %     for i in range(len(y)):
    %         if np.all(inside[i:]):
    %             settling_idx = i
    %             break
    %
    for i = 1:length(y)
        if all(inside(i:end))
            settling_idx = i;
            break;
        end
    end

    if isempty(settling_idx)
        settling_time = NaN;
    else
        settling_time = t(settling_idx);
    end

    %----------------------------------------------------------------------
    % Error
    %----------------------------------------------------------------------

    e = 1.0 - y;

    %----------------------------------------------------------------------
    % ISE - Integral of Squared Error
    %
    % Original Python:
    %
    %     ISE = np.sum(e**2) * dt
    %
    % dt is obtained from the time vector.
    %----------------------------------------------------------------------

    dt = t(2) - t(1);

    ISE = sum(e.^2) * dt;

    %----------------------------------------------------------------------
    % ITAE - Integral of Time-weighted Absolute Error
    %
    % Original Python:
    %
    %     ITAE = np.sum(np.abs(e) * t) * dt
    %----------------------------------------------------------------------

    ITAE = sum(abs(e) .* t) * dt;

    %----------------------------------------------------------------------
    % Return metrics
    %----------------------------------------------------------------------

    metrics.rise_time     = rise_time;
    metrics.overshoot     = overshoot;
    metrics.settling_time = settling_time;
    metrics.ISE           = ISE;
    metrics.ITAE          = ITAE;

end
