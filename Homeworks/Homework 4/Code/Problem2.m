% Problem2: Sketch the zero-angle configuration from the given DH table.
%
% Arael Anaya
% 10920967
% MEGN544
% 09/27/2026

clear; close all; clc;

addpath(genpath('../../../Functions'));

%% DH table (Problem 2)
% Columns: [a, d, alpha, theta]
% theta is set to 0 for the zero-angle configuration.
%   Link   a     d     alpha      theta
DH = [      1     1     pi/2       0   ;   % Link 1
            0.5   1    -pi/2       0   ;   % Link 2
            0     0     0          0   ;   % Link 3
            1     0     pi/2       0   ;   % Link 4
            0     0    -pi/2       0   ;   % Link 5
            0     0     pi/2       0  ];   % Link 6

n = size(DH, 1); % number of links

%% TODO: Forward kinematics
% Use dhTransform(a, d, alpha, theta) to build each link transform,
% chain them together to get 0T_i for i = 1..n, and pull out whatever
% points/frames you want to plot (e.g. the origin of each link frame).


%% Plot setup
figure('Name', 'Problem 2: Zero-Angle Configuration');
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Zero-Angle Configuration');
view(3);

%% TODO: Plot the manipulator
% Use the points/frames computed above to plot the manipulator, e.g.:
%   plot3(x, y, z, '-o');
% and/or plot the coordinate frame axes at each joint.
