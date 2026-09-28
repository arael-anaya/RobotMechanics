% dualQuat2PQ: Position and quaternion from a dual quaternion.
%
% [pos, quat] = dualQuat2PQ(dual_quat)
% Position and quaternion from a dual quaternion.
%
% pos = [3x1] position
% quat = [4x1] unit quaternion [q0;q_vec]
%
% dual_quat = struct with members rot and disp
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function [pos, quat] = dualQuat2PQ(dual_quat)
    quat = dual_quat.rot;

    quat_conj = [quat(1); -quat(2:4)];
    t_quat = 2 * multiplyQuat(dual_quat.disp, quat_conj);
    pos = t_quat(2:4);
end
