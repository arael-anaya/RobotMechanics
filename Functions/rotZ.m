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
    R = [   cos(theta) -sin(theta)  0 , 
            sin(theta) , cos(theta) 0 ,
                0           0        1]

    return R
end
