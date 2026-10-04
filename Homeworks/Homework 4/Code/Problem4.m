% Problem4: 0T6 at theta = 0 and pi/6
%
% Arael Anaya
% 10920967
% MEGN544
% 09/28/2026

clear; close all; clc;

addpath(genpath('../../../Functions'));

DH = [1     1     pi/2       0   ;
      0.5   1    -pi/2       0   ;
      0     0     0          0   ;
      1     0     pi/2       0   ;
      0     0    -pi/2       0   ;
      0     0     pi/2       0  ];

T = fkChain(DH);

figure('Name', 'Problem 4: Zero-Angle Configuration');
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Problem 4: Zero-Angle Configuration');
view(3);

origins = squeeze(T(1:3,4,:))';
plot3(origins(:,1), origins(:,2), origins(:,3), '-o');

axisLen = 0.2;
for i = 1:size(T,3)
    Ti = T(:,:,i);
    p = Ti(1:3,4);
    quiver3(p(1), p(2), p(3), Ti(1,1), Ti(2,1), Ti(3,1), axisLen, 'r');
    quiver3(p(1), p(2), p(3), Ti(1,2), Ti(2,2), Ti(3,2), axisLen, 'g');
    quiver3(p(1), p(2), p(3), Ti(1,3), Ti(2,3), Ti(3,3), axisLen, 'b');
end

disp('A:'); disp(T(:,:,end));

t = pi/6;
DH(:,4) = t;
T = fkChain(DH);
disp('B:'); disp(T(:,:,end));
