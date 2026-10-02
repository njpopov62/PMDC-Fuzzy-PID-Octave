function [dKp_norm, dKi_norm, dKd_norm] = infer(tuner, e_norm, de_norm)
%==========================================================================
% infer.m
%
% Fuzzy inference for the PID gain adjustments.
%
% Direct GNU Octave translation of the original Python:
%
%     def infer(self, e_norm, de_norm):
%
% The algorithm is:
%
%   1. Fuzzify e_norm and de_norm.
%   2. Calculate each rule firing strength:
%
%          w = min(mu_e, mu_de)
%
%   3. Use singleton consequence centers:
%
%          NB = -1
%          NM = -0.5
%          Z  =  0
%          PM =  0.5
%          PB =  1
%
%   4. Calculate the weighted average for dKp, dKi and dKd.
%
% No algorithmic modification is made.
%==========================================================================

    %----------------------------------------------------------------------
    % Fuzzify the normalized error and normalized derivative
    %----------------------------------------------------------------------

    mu_e  = fuzzify(e_norm);
    mu_de = fuzzify(de_norm);

    %----------------------------------------------------------------------
    % Initialize weighted sums
    %----------------------------------------------------------------------

    sum_w       = 0.0;
    sum_dKp     = 0.0;
    sum_dKi     = 0.0;
    sum_dKd     = 0.0;

    %----------------------------------------------------------------------
    % Process all fuzzy rules
    %
    % Each row of tuner.rules contains:
    %
    %   column 1 = error antecedent
    %   column 2 = derivative antecedent
    %   column 3 = dKp consequence
    %   column 4 = dKi consequence
    %   column 5 = dKd consequence
    %----------------------------------------------------------------------

    for i = 1:size(tuner.rules, 1)

        % Rule antecedent labels
        e_label  = tuner.rules{i, 1};
        de_label = tuner.rules{i, 2};

        % Consequence labels
        dKp_label = tuner.rules{i, 3};
        dKi_label = tuner.rules{i, 4};
        dKd_label = tuner.rules{i, 5};

        % Membership values
        mu_e_value  = mu_e.(e_label);
        mu_de_value = mu_de.(de_label);

        % Rule firing strength
        w = min(mu_e_value, mu_de_value);

        % Singleton consequence values
        dKp_value = get_center(tuner, dKp_label);
        dKi_value = get_center(tuner, dKi_label);
        dKd_value = get_center(tuner, dKd_label);

        % Accumulate weighted values
        sum_w   = sum_w   + w;
        sum_dKp = sum_dKp + w*dKp_value;
        sum_dKi = sum_dKi + w*dKi_value;
        sum_dKd = sum_dKd + w*dKd_value;

    end

    %----------------------------------------------------------------------
    % Weighted-average defuzzification
    %----------------------------------------------------------------------

    if sum_w > 0
        dKp_norm = sum_dKp / sum_w;
        dKi_norm = sum_dKi / sum_w;
        dKd_norm = sum_dKd / sum_w;
    else
        % Equivalent to the zero-output case when no rule fires.
        dKp_norm = 0.0;
        dKi_norm = 0.0;
        dKd_norm = 0.0;
    end

end


%==========================================================================
% Local helper function
%
% Obtain the singleton center corresponding to a linguistic label.
%
% NB -> -1
% NM -> -0.5
% Z  ->  0
% PM ->  0.5
% PB ->  1
%==========================================================================

function value = get_center(tuner, label)

    idx = find(strcmp(tuner.labels, label), 1);

    if isempty(idx)
        error('Unknown fuzzy label: %s', label);
    end

    value = tuner.centers(idx);

end
