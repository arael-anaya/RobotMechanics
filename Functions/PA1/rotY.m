% rotY: Rotation matrix about the Y axis.
%
% R = rotY(theta)
% Rotation matrix about the Y axis.
%
% R = [3x3] rotation matrix
%
% theta = rotation angle (rad)
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function R = rotY(theta)
    c = cos(theta);
    s = sin(theta);
    R = [ c 0 s;
          0 1 0;
         -s 0 c];
end
