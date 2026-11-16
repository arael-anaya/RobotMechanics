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
    N = numel(linkList);
    computeDot = nargin >= 3 && ~isempty(paramRateList);

    T = eye(4,4);
    d_list = zeros(3,N);
    z_list = zeros(3,N);
    v_list = zeros(3,N);
    w_list = zeros(3,N);
    w_total = [0;0;0];
    v_total = [0;0;0];
    
    for i = 1:N
        link = linkList(i); 
        z_list(:,i) = T(1:3,3);
        d_list(:,i) = T(1:3,4);
        w_list(:,i) = w_total;
        v_list(:,i) = v_total;      
        if link.isRotary == 1
            T_this = dhTransform(link.a,link.d,link.alpha, paramList(i) - link.offset);
        elseif link.isRotary == 0
            T_this = dhTransform(link.a, paramList(i) - link.offset, link.alpha, link.theta);
        else
            T_this = dhTransform(link.a, link.d, link.alpha, link.theta);
        end
        if computeDot
            if link.isRotary == 1
                w_total = w_total +  paramRateList(i) * z_list(:,i);
            end
            r = T(1:3,1:3) * T_this(1:3,4);
            v_total = v_total + cross(w_total, r);

            if link.isRotary == 0
                v_total = v_total +  paramRateList(i) * z_list(:,i);
            end
        end
        T = T * T_this;
    end

    d_total = T(1:3,4);
    Jv = zeros(6,N);
    JvDot = [];
    if computeDot, JvDot = zeros(6,N); end

    for i = 1:N
        z  = z_list(:,i);
        dd = d_total - d_list(:,i);
        if linkList(i).isRotary == 1
            Jv(:,i) = [cross(z, dd); z];
            if computeDot
                zdot = cross(w_list(:,i), z);
                JvDot(:,i) = [cross(zdot, dd) + cross(z, v_total - v_list(:,i)); ...
                              zdot];
            end
        elseif linkList(i).isRotary == 0
            Jv(:,i) = [z; zeros(3,1)];
            if computeDot
                JvDot(:,i) = [cross(w_list(:,i), z); zeros(3,1)];
            end
        end 
    end
end

