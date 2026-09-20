% rpy2Rot: Rotation matrix from roll-pitch-yaw, R = rotZ(yaw)*rotY(pitch)*rotX(roll).
%
% R = rpy2Rot(roll, pitch, yaw)
% Rotation matrix from roll-pitch-yaw, R = rotZ(yaw)*rotY(pitch)*rotX(roll).
%
% R = [3x3] rotation matrix
%
% roll = rotation about X (rad)
% pitch = rotation about Y (rad)
% yaw = rotation about Z (rad)
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function R = rpy2Rot(roll, pitch, yaw)
    R = rotZ(yaw) * rotY(pitch) * rotX(roll);
end
