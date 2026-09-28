% pq2DualQuat: Dual quaternion from a position and quaternion.
%
% dual_quat = pq2DualQuat(pos, quat)
% Dual quaternion from a position and quaternion.
%
% dual_quat = struct with rot [4x1] unit quaternion and disp [4x1] quaternion
%
% pos = [3x1] position
% quat = [4x1] unit quaternion [q0;q_vec]
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function dual_quat = pq2DualQuat(pos, quat)
    dual_quat.rot = quat;

    dual_quat.disp = 0.5 * multiplyQuat([0; pos(:)], dual_quat.rot);
end
