% rot2Quat: Quaternion from a rotation matrix.
%
% Q = rot2Quat(R)
% Quaternion from a rotation matrix.
%
% Q = [4x1] quaternion [q0;q_vec]
%
% R = [3x3] rotation matrix
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function Q = rot2Quat(R)

    q_0 = sqrt(max(0, 1 + trace(R))) / 2;

    if q_0 > 1e-6
        q1 = (R(3,2) - R(2,3)) / (4 * q_0);
        q2 = (R(1,3) - R(3,1)) / (4 * q_0);
        q3 = (R(2,1) - R(1,2)) / (4 * q_0);
    else
        % near 180 degree rotation: q_0 ~ 0, so divide using the
        % largest diagonal term instead to avoid dividing by ~0
        [~, i] = max(diag(R));
        if i == 1
            q1 = sqrt(max(0, 1 + R(1,1) - R(2,2) - R(3,3))) / 2;
            q_0 = (R(3,2) - R(2,3)) / (4 * q1);
            q2 = (R(1,2) + R(2,1)) / (4 * q1);
            q3 = (R(1,3) + R(3,1)) / (4 * q1);
        elseif i == 2
            q2 = sqrt(max(0, 1 - R(1,1) + R(2,2) - R(3,3))) / 2;
            q_0 = (R(1,3) - R(3,1)) / (4 * q2);
            q1 = (R(1,2) + R(2,1)) / (4 * q2);
            q3 = (R(2,3) + R(3,2)) / (4 * q2);
        else
            q3 = sqrt(max(0, 1 - R(1,1) - R(2,2) + R(3,3))) / 2;
            q_0 = (R(2,1) - R(1,2)) / (4 * q3);
            q1 = (R(1,3) + R(3,1)) / (4 * q3);
            q2 = (R(2,3) + R(3,2)) / (4 * q3);
        end
    end

    Q = [q_0;q1;q2;q3];

end
