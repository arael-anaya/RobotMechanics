% armPositions: End-effector positions along a joint-space history.
%
% P = armPositions(q, a1, a2)
% End-effector positions along a joint-space history.
%
% P = [3 x m] end-effector positions, one column per sample
%
% q = [6 x m] joint values [theta1; theta2; d3; theta4; theta5; theta6]
% a1 = length of link 1
% a2 = length of link 2
%
% Arael Anaya
% 10920967
% MEGN544
% 10/12/2026
function P = armPositions(q, a1, a2)
    m = size(q,2);
    P = zeros(3,m);
    for k = 1:m
        DH = [a1 0 0     q(1,k);
              a2 0 0     q(2,k);
              0  q(3,k) 0 0;
              0  0  pi/2  q(4,k);
              0  0 -pi/2  q(5,k);
              0  0  0     q(6,k)];
        F = fkChain(DH);
        P(:,k) = F(1:3,4,end);
    end
end
