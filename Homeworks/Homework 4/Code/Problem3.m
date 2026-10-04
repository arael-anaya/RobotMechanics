% Problem3: Sketch the zero-angle configuration from the given DH table.
%
% Arael Anaya
% 10920967
% MEGN544
% 09/28/2026

clear; close all; clc;

addpath(genpath('../../../Functions'));

%% DH table (Problem 3)
%   Link   a     d     alpha      theta
DH = [      1     0     pi/2       0   ;   % Link 1
            1     0     pi/2       pi/2   ;   % Link 2
            1     0     0          0   ;  ];  % Link 3

n = size(DH, 1); 

T_list = zeros(4, 4, n);
for i = 1:n
    T_list(:,:,i) = dhTransform(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
end

T01 = T_list(:,:,1);
T02 = T01 * T_list(:,:,2);
T03 = T02 * T_list(:,:,3);


figure('Name', 'Problem 3: Zero-Angle Configuration');
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Zero-Angle Configuration');
view(3);

origins = [0 0 0; T01(1:3,4)'; T02(1:3,4)'; T03(1:3,4)'];
plot3(origins(:,1), origins(:,2), origins(:,3), '-o');

axisLen = 0.2;
frames = {eye(4), T01, T02, T03};
for i = 1:numel(frames)
    T = frames{i};
    p = T(1:3,4);
    xAxis = T(1:3,1);
    yAxis = T(1:3,2);
    zAxis = T(1:3,3);
    quiver3(p(1), p(2), p(3), xAxis(1), xAxis(2), xAxis(3), axisLen, 'r');
    quiver3(p(1), p(2), p(3), yAxis(1), yAxis(2), yAxis(3), axisLen, 'g');
    quiver3(p(1), p(2), p(3), zAxis(1), zAxis(2), zAxis(3), axisLen, 'b');
end
