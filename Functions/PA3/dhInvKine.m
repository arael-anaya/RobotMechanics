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
    
    iter = 1;
    TOL = 1e-10;
    Tc = dhFwdKine(linkList, paramListGuess);
    err = calculate_error(desTransform, Tc);
    paramList = paramListGuess;
    while norm(err) > TOL && iter < 100
        Jv = velocityJacobian(linkList, paramList);
        paramList = paramList + pinv(Jv) * err;
        Tc = dhFwdKine(linkList, paramList);
        err = calculate_error(desTransform, Tc);
        iter = iter + 1;
    end
    
    for i = 1:numel(linkList)
        if linkList(i).isRotary == 1
            paramList(i) = atan2(sin(paramList(i)), cos(paramList(i)));
        end
    end

    error = norm(err);

    function errr = calculate_error(Td, Tc)
        p_error = Td(1:3,4) - Tc(1:3,4);
        R_error = Td(1:3,1:3)* (Tc(1:3,1:3))';

        errr = [p_error; rot2AngleAxis(R_error)];
    end
end
