% rotX: Rotation matrix about the X axis.
%
% R = rotX(theta)
% Rotation matrix about the X axis.
%
% R = [3x3] rotation matrix
%
% theta = rotation angle (rad)
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function R = rotX(theta)
    c = cos(theta);
    s = sin(theta);
    R = [1 0  0;
         0 c -s;
         0 s  c];
end
