% multiplyDualQuat: Dual quaternion product (chained transformation).
%
% dual_quat = multiplyDualQuat(dual_quat_left, dual_quat_right)
% Dual quaternion product (chained transformation).
%
% dual_quat = struct with members rot and disp
%
% dual_quat_left = struct with members rot and disp
% dual_quat_right = struct with members rot and disp
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function dual_quat = multiplyDualQuat(dual_quat_left, dual_quat_right)
    dual_quat.rot = multiplyQuat(dual_quat_left.rot, dual_quat_right.rot);
    dual_quat.disp = multiplyQuat(dual_quat_left.rot, dual_quat_right.disp) ...
                    + multiplyQuat(dual_quat_left.disp, dual_quat_right.rot);
end
