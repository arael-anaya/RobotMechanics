% rotZ: Rotation matrix about the Z axis.
%
% R = rotZ(theta)
% Rotation matrix about the Z axis.
%
% R = [3x3] rotation matrix
%
% theta = rotation angle (rad)
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function R = rotZ(theta)
    c = cos(theta);
    s = sin(theta);
    R = [c -s 0;
         s  c 0;
         0  0 1];
end
