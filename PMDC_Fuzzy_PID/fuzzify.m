function mu = fuzzify(x)
%==========================================================================
% fuzzify.m
%
% Fuzzification of a normalized input variable.
%
% Direct GNU Octave translation of the original Python:
%
%     def fuzzify(x):
%         return {
%             "NB": tri_mf(x, -1.5, -1.0, -0.5),
%             "NM": tri_mf(x, -1.0, -0.5,  0.0),
%             "Z":  tri_mf(x, -0.5,  0.0,  0.5),
%             "PM": tri_mf(x,  0.0,  0.5,  1.0),
%             "PB": tri_mf(x,  0.5,  1.0,  1.5)
%         }
%
% Input:
%     x  - normalized input, normally in [-1, 1]
%
% Output:
%     mu - structure containing the five membership values
%
% No algorithmic modification is made.
%==========================================================================

    % Negative Big
    mu.NB = tri_mf(x, -1.5, -1.0, -0.5);

    % Negative Medium
    mu.NM = tri_mf(x, -1.0, -0.5, 0.0);

    % Zero
    mu.Z = tri_mf(x, -0.5, 0.0, 0.5);

    % Positive Medium
    mu.PM = tri_mf(x, 0.0, 0.5, 1.0);

    % Positive Big
    mu.PB = tri_mf(x, 0.5, 1.0, 1.5);

end
