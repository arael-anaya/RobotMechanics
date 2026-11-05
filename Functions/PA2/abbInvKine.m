% abbInvKine: Inverse kinematics for the 6-DOF ABB arm.
%
% [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
% Computes the joint angles that place the end effector at the desired
% pose. Of the possible IK solutions, the one closest to th_last is chosen.
% If th_last is not provided, all solutions are returned (up to 8 per joint).
%
% th1...th6 = joint angles 1 through 6 (rad); scalars if th_last is given,
%   otherwise column vectors with one entry per solution
% reachable = true if the pose can be achieved, false otherwise
%
% T_des = [4x4] desired homogeneous transform of the end effector
% th_last = [6x1] previous joint angles (rad), used to select among solutions
%   (optional)
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
    
    d1 = 0.290;  a2 = 0.270;  a3 = 0.070;  d4 = 0.302;  d6 = 0.072;
    
    reachable = false;
    
    sols = [];
    allSolutions = ~exist('th_last','var') || isempty(th_last);
    if allSolutions, th_last = zeros(6,1); end   % only used at wrist singularities
    th_last = th_last(:);
    wrapPi = @(x) atan2(sin(x), cos(x));

    p_des = T_des(1:3,4);
    R_des = T_des(1:3,1:3);

    pw = p_des - d6 * R_des(:,3);

    th1_opts = [atan2(pw(2), pw(1)) , atan2(pw(2),pw(1)) + pi];

    links03 = [ createLink(0,   d1,    -pi/2,   [],     0)   ,
                createLink(a2,  0,      0,      [],     pi/2),
                createLink(a3,  0,     -pi/2,   [],     0)];

    for t1 = th1_opts
        r = cos(t1)*pw(1) + sin(t1) * pw(2);
        L = sqrt(a3^2 + d4^2);
        s = pw(3) - d1;
        
        cosGamma = (r^2 + s^2 - a2^2 - L^2) / (2*a2*L);
        if abs(cosGamma) > 1, continue; end
        sinGamma = sqrt(1-cosGamma^2);
        gamma = atan2(sinGamma, cosGamma);

        for elbow = [1,-1]
            g = elbow * gamma;

            t3 = g - atan2(d4,a3);
            elev = atan2(s, r) + atan2(L*sin(g), a2 + L*cos(g));
            t2 = pi/2 - elev;

            H03 = dhFwdKine(links03 , [t1 t2 t3]);
            R03 = H03(1:3,1:3);

            R36 = R03' * R_des;
            for wrist = [1,-1]

                t5 = wrist * atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
                
                if abs(sin(t5)) <= 1e-9
                    % Wrist singularity: only one combination of th4, th6 is
                    % determined. Split the required change equally between them.
                    if cos(t5) > 0       % th5 = 0: th4 + th6 is fixed
                        change = wrapPi(atan2(R36(2,1), R36(1,1)) - (th_last(4) + th_last(6)));
                        t4 = th_last(4) + change/2;
                        t6 = th_last(6) + change/2;
                    else                 % th5 = +-pi: th4 - th6 is fixed
                        change = wrapPi(atan2(-R36(2,1), -R36(1,1)) - (th_last(4) - th_last(6)));
                        t4 = th_last(4) + change/2;
                        t6 = th_last(6) - change/2;
                    end
                else
                    t4 = atan2(-wrist*R36(2,3) , -wrist*R36(1,3));
                    t6 = atan2(-wrist*R36(3,2) , wrist*R36(3,1));
                end

                sols(:,end+1) = [t1;t2;t3;t4;t5;t6];
            end
        end
    end
    reachable = ~isempty(sols);
    if ~reachable, th1=NaN;th2=NaN;th3=NaN;th4=NaN;th5=NaN;th6=NaN; return; end
    
    if allSolutions
        % No th_last: return every solution (up to 8) as column vectors
        sols = wrapPi(sols);
        th1 = sols(1,:)'; th2 = sols(2,:)'; th3 = sols(3,:)';
        th4 = sols(4,:)'; th5 = sols(5,:)'; th6 = sols(6,:)';
        return;
    end
    diffs = wrapPi(sols - th_last);
    [~, idx] = min(vecnorm(diffs));

    % Return the angles nearest th_last (not forced into [-pi, pi])
    best = th_last + diffs(:,idx);
    th1 = best(1); th2 = best(2); th3 = best(3);
    th4 = best(4); th5 = best(5); th6 = best(6);
end
