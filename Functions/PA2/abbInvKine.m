% abbInvKine: Inverse kinematics for the 6-DOF ABB arm.
%
% [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
% Computes the joint angles that place the end effector at the desired
% pose. If th_last is given, the single solution closest to th_last is
% returned. If th_last is not given, every solution is returned.
%
% th1...th6 = joint angles 1 through 6 (rad). Scalars if th_last is given,
%             otherwise 8x1 vectors (one entry for each possible solution)
% reachable = true if the transform can be achieved, false otherwise. If it
%             cannot be achieved the closest (arm fully stretched) pose is
%             returned
%
% T_des = [4x4] desired homogeneous transform of the end effector
% th_last = [6x1] previous joint angles (rad), used to select among solutions
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)

    d1 = 0.290;  a2 = 0.270;  a3 = 0.070;  d4 = 0.302;  d6 = 0.072;
    singTol = 1e-6;

    haveLast = exist('th_last', 'var') && ~isempty(th_last);
    if haveLast
        th_last = th_last(:);
        last4 = th_last(4);
        last6 = th_last(6);
    else
        last4 = 0;
        last6 = 0;
    end

    reachable = true;
    sols = zeros(6, 0);

    p_des = T_des(1:3,4);
    R_des = T_des(1:3,1:3);

    % wrist center
    pw = p_des - d6 * R_des(:,3);

    th1_opts = [atan2(pw(2), pw(1)), atan2(pw(2), pw(1)) + pi];

    links03 = [createLink(0,   d1,    -pi/2,   [],     0);
               createLink(a2,  0,      0,      [],     pi/2);
               createLink(a3,  0,     -pi/2,   [],     0)];

    L = sqrt(a3^2 + d4^2);

    for t1 = th1_opts
        r = cos(t1)*pw(1) + sin(t1)*pw(2);
        s = pw(3) - d1;

        cosGamma = (r^2 + s^2 - a2^2 - L^2) / (2*a2*L);
        if abs(cosGamma) > 1
            % out of reach, use the nearest pose (arm fully stretched/folded)
            reachable = false;
            cosGamma = sign(cosGamma);
        end
        gamma = atan2(sqrt(1 - cosGamma^2), cosGamma);

        for elbow = [1, -1]
            g = elbow * gamma;

            t3 = g - atan2(d4, a3);
            elev = atan2(s, r) + atan2(L*sin(g), a2 + L*cos(g));
            t2 = pi/2 - elev;

            H03 = dhFwdKine(links03, [t1 t2 t3]);
            R36 = H03(1:3,1:3)' * R_des;

            for wrist = [1, -1]
                t5 = wrist * atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));

                if abs(sin(t5)) <= singTol
                    % wrist singularity: only a combination of th4 and th6
                    % is determined, split the change evenly from th_last
                    if R36(3,3) > 0
                        % th5 = 0: th4 + th6 is fixed
                        delta = wrapToPiLocal(atan2(R36(2,1), R36(1,1)) - (last4 + last6));
                        t4 = last4 + delta/2;
                        t6 = last6 + delta/2;
                    else
                        % th5 = pi: th4 - th6 is fixed
                        delta = wrapToPiLocal(atan2(-R36(2,1), -R36(1,1)) - (last4 - last6));
                        t4 = last4 + delta/2;
                        t6 = last6 - delta/2;
                    end
                else
                    t4 = atan2(-wrist*R36(2,3), -wrist*R36(1,3));
                    t6 = atan2(-wrist*R36(3,2),  wrist*R36(3,1));
                end

                sols(:,end+1) = [t1; t2; t3; t4; t5; t6]; %#ok<AGROW>
            end
        end
    end

    if haveLast
        % shift each angle by multiples of 2*pi to be nearest th_last, then
        % choose the solution with the smallest overall change
        sols = th_last + wrapToPiLocal(sols - th_last);
        [~, idx] = min(vecnorm(sols - th_last));
        sols = sols(:, idx);
    else
        sols = wrapToPiLocal(sols);
    end

    th1 = sols(1,:)';
    th2 = sols(2,:)';
    th3 = sols(3,:)';
    th4 = sols(4,:)';
    th5 = sols(5,:)';
    th6 = sols(6,:)';
end

% wrapToPiLocal: wrap angles (rad) into the range [-pi, pi]
function ang = wrapToPiLocal(ang)
    ang = atan2(sin(ang), cos(ang));
end
