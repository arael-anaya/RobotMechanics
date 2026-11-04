% dhTransform: Homogeneous transform from DH parameters.
%
% H = dhTransform(a, d, alpha, theta)
% Homogeneous transform from DH parameters.
%
% H = [4x4] homogeneous transform
%
% a = link length
% d = link offset
% alpha = link twist (rad)
% theta = joint angle (rad)
%
% Arael Anaya
% 10920967
% MEGN544
% 09/28/2026
function H = dhTransform(a, d, alpha, theta)
    R = rotZ(theta) * rotX(alpha);
    p = [a*cos(theta); a*sin(theta); d];

    H = [R, p;
         0, 0, 0, 1];
end
