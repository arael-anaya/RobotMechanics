% quat2Rot: Rotation matrix from a quaternion.
%
% R = quat2Rot(Q)
% Rotation matrix from a quaternion.
%
% R = [3x3] rotation matrix
%
% Q = [4x1] quaternion [q0;q_vec]
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function R = quat2Rot(Q)
    Q = Q(:);
    q_0 = Q(1);
    q = Q(2:4);

    R = (q_0^2 - dot(q,q))*eye(3) + 2*q_0*skew(q)+2 * q * transpose(q);

% TODO: implement
end
