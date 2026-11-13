% velocityJacobian: Velocity Jacobian of a serial manipulator.
%
% [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
% Returns the velocity Jacobian of the manipulator given an array of links
% created by createLink and the current joint variables. If paramRateList
% is not provided, JvDot is returned as [].
%
% Jv = velocity Jacobian [6xN]
% JvDot = time derivative of the velocity Jacobian [6xN]
%
% linkList = array of links created by createLink
% paramList = current theta or d values for the joints, according to the
%             robot's encoders (Nx1)
% paramRateList = current thetaDot and dDot values for the joints (Nx1)
%
% Arael Anaya
% 10920967
% MEGN544
% 11/13/2026
function [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
    % YOUR FUNCTION CODE GOES HERE
end
