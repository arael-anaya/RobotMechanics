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
% TODO: implement
    q_0 = sqrt((1+ trace(R)) / 4);

    q1 = (R(3,2) - R(2 ,3)) / (4 * q_0);
    q2 = (R(1,3) - R(3 ,1)) / (4 * q_0);
    q3 = (R(2,1) - R(1 ,2)) / (4 * q_0);

    Q = [q_0;q1;q2;q3];


end
