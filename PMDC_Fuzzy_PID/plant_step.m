function x_next = plant_step(x, Va, dt, Ra, La, J, B, Kt, Ke)
%==========================================================================
% plant_step.m
%
% PMDC motor state update.
%
% This is a direct GNU Octave translation of the original Python:
%
%     def plant_step(x, Va, dt):
%
% State:
%     x(1) = omega  - angular velocity
%     x(2) = ia     - armature current
%
% Motor equations:
%
%     J  * domega/dt = Kt*ia - B*omega
%
%     La * dia/dt    = Va - Ra*ia - Ke*omega
%
% Numerical integration:
%
%     Forward Euler
%
% No algorithmic modification is made.
%==========================================================================

    % Current state
    omega = x(1);
    ia    = x(2);

    % State derivatives
    domega = (Kt*ia - B*omega) / J;

    dia = (Va - Ra*ia - Ke*omega) / La;

    % Forward Euler integration
    omega_next = omega + dt*domega;
    ia_next    = ia + dt*dia;

    % Return next state
    x_next = [omega_next;
              ia_next];

end

