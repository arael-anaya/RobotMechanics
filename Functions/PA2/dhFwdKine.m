% dhFwdKine: Forward kinematics of a serial arm using DH parameters.
%
% H = dhFwdKine(linkList, paramList)
% Multiplies the DH transform of each link, in order, to get the pose of
% the end effector in the base frame. Each joint variable in paramList is
% used as theta if the link's theta is empty (revolute), otherwise as d
% (prismatic).
%
% H = [4x4] homogeneous transform from the base frame to the end effector
%
% linkList = array of links (struct array), each created with createLink
% paramList = joint variable for each link (rad or m)
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function H = dhFwdKine(linkList, paramList)
    H = eye(4);
    for i = 1:numel(linkList)
        link = linkList(i);
        param = paramList(i);
        if link.isRotary == 1
            linkTransform = dhTransform(link.a, link.d, link.alpha, param - link.offset);
        elseif link.isRotary == 0
            linkTransform = dhTransform(link.a, param - link.offset, link.alpha, link.theta);
        else
             linkTransform = dhTransform(link.a, link.d, link.alpha, link.theta);            
        end
        H = H * linkTransform;
    end

end
