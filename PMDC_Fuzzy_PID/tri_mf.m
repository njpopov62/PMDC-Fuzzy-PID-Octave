function mu = tri_mf(x, a, b, c)
%==========================================================================
% tri_mf.m
%
% Triangular membership function.
%
% Direct GNU Octave translation of the original Python:
%
%     def tri_mf(x, a, b, c):
%         if x <= a or x >= c:
%             return 0.0
%         if x == b:
%             return 1.0
%         if x < b:
%             return (x-a)/(b-a+1e-12)
%         return (c-x)/(c-b+1e-12)
%
% No algorithmic modification is made.
%==========================================================================

    % Outside the triangular membership function
    if x <= a || x >= c
        mu = 0.0;
        return;
    end

    % Peak of the triangle
    if x == b
        mu = 1.0;
        return;
    end

    % Rising edge
    if x < b
        mu = (x - a) / (b - a + 1e-12);
        return;
    end

    % Falling edge
    mu = (c - x) / (c - b + 1e-12);

end
