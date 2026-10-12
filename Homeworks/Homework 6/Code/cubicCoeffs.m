% cubicCoeffs: Cubic polynomial coefficients for a rest-to-rest trajectory.
%
% C = cubicCoeffs(q0, qf, t0, tf)
% Cubic polynomial coefficients for a rest-to-rest trajectory.
%
% C = [n x 4] coefficients per joint, q(t) = C1 + C2*tau + C3*tau^2 + C4*tau^3
%     where tau = t - t0
%
% q0 = [n x 1] initial joint values
% qf = [n x 1] final joint values
% t0 = initial time (s)
% tf = final time (s)
%
% Arael Anaya
% 10920967
% MEGN544
% 10/12/2026
function C = cubicCoeffs(q0, qf, t0, tf)
    T = tf - t0;
    dq = qf - q0;
    C = [q0, zeros(size(q0)), 3*dq/T^2, -2*dq/T^3];