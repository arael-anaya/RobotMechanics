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
    R = [cos(theta)  0 , sin(theta), 
            0 ,      1      0,
        -sin(theta)  0   cos(theta)]
    return R
end
