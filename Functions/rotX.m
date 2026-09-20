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
    R = [   1       0            0,
            0    cos(theta) -sin(theta), 
            0    sin(theta) , cos(theta)]

    return R
end
