% angleAxis2Rot: Rotation matrix from an angle-times-axis vector.
%
% R = angleAxis2Rot(omega)
% Rotation matrix from an angle-times-axis vector.
%
% R = [3x3] rotation matrix
%
% omega = [3x1] theta*k (rad)
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function L = createLink(a, d, alpha, theta, offset, centOfMass, mass, inertia)
    L = [a, d, alpha, theta, offset, centOfMass, mass, inertia];
end
