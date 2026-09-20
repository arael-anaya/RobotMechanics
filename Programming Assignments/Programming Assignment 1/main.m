% main: Driver script for Programming Assignment 1 (forward kinematics tests).
%
% Adds the shared Functions folder to the MATLAB path so every function
% there can be called from this script.
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026

clear; clc;

% Functions/ is two levels above this script
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'Functions'));

% Test calls go below
