% cubicInterp: Evaluate cubic polynomial trajectories at given times.
%
% Q = cubicInterp(C, t0, tvec)
% Evaluate cubic polynomial trajectories at given times.
%
% Q = [n x m] joint values, one column per time in tvec
%
% C = [n x 4] polynomial coefficients from cubicCoeffs
% t0 = initial time the coefficients are referenced to (s)
% tvec = [1 x m] times at which to evaluate (s)
%
% Arael Anaya
% 10920967
% MEGN544
% 10/12/2026
function Q = cubicInterp(C, t0, tvec)
    tau = tvec - t0;
    Q = C(:,1) + C(:,2).*tau + C(:,3).*tau.^2 + C(:,4).*tau.^3;