% angleAxis2Rot: Rotation matrix from an angle-times-axis vector.
%
% R = angleAxis2Rot(omega)
% Rotation matrix from an angle-times-axis vector.
%
% R = [3x3] rotation matrix
%
% omega = [3x1] theta*k (rad)
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function H = dhFwdKine(linkList, paramList)
    H = eye(4);
    for i = 1:numel(linkList)
        link = linkList{i};
        param = paramList(i);
        if isempty(list(4))
            linkTransform = dhTransform(link(1), link(2), link(3), param);
        else
            linkTransform = dhTransform(link(1), param, link(3), link(4));
        end
            H = finalTransform * linkTransform;
    end

end
