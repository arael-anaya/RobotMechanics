% Problem1Figure: Sketch the zero-angle configuration of the ABB arm from the given DH table.
%
% Arael Anaya
% 10920967
% MEGN544
% 10/03/2026

clear; close all; clc;

addpath(genpath('../../../Functions'));

d1 = 0.5;  a2 = 1.0;  a3 = 0.2;  d4 = 1.0;  d6 = 0.3;

DH = [      0     d1    pi/2       0   ;
            a2    0     0          0   ;
            a3    0    -pi/2       0   ;
            0     d4    pi/2       0   ;
            0     0    -pi/2       0   ;
            0     0     0          0  ];
DH(6,2) = d6;

T = fkChain(DH);

fig = figure('Name', 'Problem 1: Zero-Angle Configuration', 'Color', 'w');
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Zero-Angle Configuration of the ABB Arm');
view(3);

origins = squeeze(T(1:3,4,:))';
plot3(origins(:,1), origins(:,2), origins(:,3), 'k-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'k');

axisLen = 0.25;
for i = 1:size(T,3)
    p = T(1:3,4,i);
    quiver3(p(1), p(2), p(3), T(1,1,i), T(2,1,i), T(3,1,i), axisLen, 'r', 'LineWidth', 1.5);
    quiver3(p(1), p(2), p(3), T(1,2,i), T(2,2,i), T(3,2,i), axisLen, 'g', 'LineWidth', 1.5);
    quiver3(p(1), p(2), p(3), T(1,3,i), T(2,3,i), T(3,3,i), axisLen, 'b', 'LineWidth', 1.5);
    % Frames 4 and 5 share an origin, so they get one combined label offset sideways
    if i == 5
        text(p(1)+0.04, p(2)+0.2, p(3)+0.04, '\{4\},\{5\}');
    elseif i ~= 6
        text(p(1)+0.04, p(2)+0.04, p(3)+0.04, sprintf('\\{%d\\}', i-1));
    end
end

exportgraphics(fig, '../Latex/Figures/Problem1Figure.png', 'Resolution', 200);
