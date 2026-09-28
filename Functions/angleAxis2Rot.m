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
function R = angleAxis2Rot(omega)
    theta = norm(omega);
    if theta < 1e-12
        R = eye(3);
        return;
    end    
    k = (1/theta) * omega;
    R = cos(theta) * eye(3) + sin(theta) * skew(k) + (1 - cos(theta)) * (k * k');
end
