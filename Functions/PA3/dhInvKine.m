% dhInvKine: Numerical inverse kinematics of a serial manipulator.
%
% [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
% Returns the parameter list, according to the robot's encoders, necessary
% to achieve a desired homogeneous transform, and the residual error.
% All rotary joints are wrapped to the -pi to pi range.
%
% paramList = joint parameters that achieve the desired transform (Nx1)
% error = residual error of the solution
%
% linkList = list of the joint parameters created by createLink
% desTransform = desired homogeneous transform [4x4]
% paramListGuess = initial guess at the parameters, according to the
%                  robot's encoders (Nx1)
%
% Arael Anaya
% 10920967
% MEGN544
% 11/13/2026
function [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
    % YOUR FUNCTION CODE GOES HERE
end
