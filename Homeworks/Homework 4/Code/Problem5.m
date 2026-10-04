% Problem5: Effect of link offset d on 0T3
%
% Arael Anaya
% 10920967
% MEGN544
% 09/28/2026

clear; close all; clc;

addpath(genpath('../../../Functions'));

DH = [1     0     pi/2       0   ;
      1     0     pi/2       pi/2;
      1     0     0          0  ];

T = fkChain(DH);

figure('Name', 'Problem 5: Zero-Angle Configuration');
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Problem 5: Zero-Angle Configuration');
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

d = 1;

DH = [1     d     pi/2       0   ;
      1     0     pi/2       pi/2;
      1     0     0          0  ];
T = fkChain(DH);
disp('A:'); disp(T(:,:,end));

DH = [1     0     pi/2       0   ;
      1     d     pi/2       pi/2;
      1     0     0          0  ];
T = fkChain(DH);
disp('B:'); disp(T(:,:,end));

DH = [1     0     pi/2       0   ;
      1     0     pi/2       pi/2;
      1     d     0          0  ];
T = fkChain(DH);
disp('C:'); disp(T(:,:,end));
