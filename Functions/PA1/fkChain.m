% fkChain: Cumulative forward-kinematics transforms from a DH table.
%
% T = fkChain(DH)
% Cumulative forward-kinematics transforms from a DH table.
%
% T = [4x4x(n+1)] stack of transforms, T(:,:,1) = base frame,
%     T(:,:,i+1) = 0Ti for link i
%
% DH = [n x 4] table of [a, d, alpha, theta] rows
%
% Arael Anaya
% 10920967
% MEGN544
% 09/28/2026
function T = fkChain(DH)
    n = size(DH, 1);
    T = zeros(4, 4, n+1);
    T(:,:,1) = eye(4);
    for i = 1:n
        Ti = dhTransform(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
        T(:,:,i+1) = T(:,:,i) * Ti;
    end
end
