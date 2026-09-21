% rot2ZYZ: ZYZ Euler angles from a rotation matrix.
%
% [phi, theta, psi] = rot2ZYZ(R)
% ZYZ Euler angles from a rotation matrix, R = Rz(phi)*Ry(theta)*Rz(psi).
%
% phi = [2x1] first z rotation solutions, +sqrt on top (rad)
% theta = [2x1] y rotation solutions, +sqrt on top (rad)
% psi = [2x1] last z rotation solutions, +sqrt on top (rad); [0;0] if degenerate
%
% R = [3x3] rotation matrix
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function [phi, theta, psi] = rot2ZYZ(R)
    st = sqrt(R(1,3)^2 + R(2,3)^2);
    if st < 1e-9
        % Degenerate (theta = 0 or pi): only phi +/- psi is determined, so set psi = 0
        if R(3,3) > 0
            theta = 0 * [1; 1];
            phi = atan2(R(2,1), R(1,1)) * [1; 1];
        else
            theta = pi * [1; 1];
            phi = atan2(-R(2,1), -R(1,1)) * [1; 1];
        end
        psi = [0; 0];
    else
        sts = [st; -st];
        theta = atan2(sts, R(3,3));
        phi = atan2(R(2,3) ./ sts, R(1,3) ./ sts);
        psi = atan2(R(3,2) ./ sts, -R(3,1) ./ sts);
    end
end
