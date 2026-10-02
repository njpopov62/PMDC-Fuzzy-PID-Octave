function tuner = FuzzyTuner()
%==========================================================================
% FuzzyTuner.m
%
% Initialization of the fuzzy PID tuner.
%
% Direct GNU Octave translation of the original Python FuzzyTuner class.
%
% Linguistic labels:
%
%     NB = Negative Big
%     NM = Negative Medium
%     Z  = Zero
%     PM = Positive Medium
%     PB = Positive Big
%
% The original Python rule table is preserved exactly.
%
% No algorithmic modification is made.
%==========================================================================

    %----------------------------------------------------------------------
    % Membership-function labels
    %----------------------------------------------------------------------

    tuner.labels = {'NB', 'NM', 'Z', 'PM', 'PB'};

    %----------------------------------------------------------------------
    % Universe centers
    %
    % Original Python:
    %
    %     self.centers = np.array([-1, -0.5, 0, 0.5, 1])
    %----------------------------------------------------------------------

    tuner.centers = [-1.0, -0.5, 0.0, 0.5, 1.0];

    %----------------------------------------------------------------------
    % Fuzzy rule table
    %
    % Each rule has:
    %
    %     antecedent  = {error_label, derivative_label}
    %     consequence = {dKp_label, dKi_label, dKd_label}
    %
    % Original Python rules:
    %
    % ('NB','NB') -> ('PB','NB','NB')
    % ('NM','NM') -> ('PM','NM','Z')
    % ('Z','Z')   -> ('Z','Z','Z')
    % ('PM','PM') -> ('NM','PM','PM')
    % ('PB','PB') -> ('NB','PB','PB')
    % ('NB','Z')  -> ('PM','NM','Z')
    % ('Z','NB')  -> ('PM','NM','NM')
    % ('PB','Z')  -> ('NM','PM','Z')
    % ('Z','PB')  -> ('NM','PM','PM')
    %----------------------------------------------------------------------

    tuner.rules = {
        'NB', 'NB', 'PB', 'NB', 'NB';
        'NM', 'NM', 'PM', 'NM', 'Z';
        'Z',  'Z',  'Z',  'Z',  'Z';
        'PM', 'PM', 'NM', 'PM', 'PM';
        'PB', 'PB', 'NB', 'PB', 'PB';
        'NB', 'Z',  'PM', 'NM', 'Z';
        'Z',  'NB', 'PM', 'NM', 'NM';
        'PB', 'Z',  'NM', 'PM', 'Z';
        'Z',  'PB', 'NM', 'PM', 'PM'
    };

end
