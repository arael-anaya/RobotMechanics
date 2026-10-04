% ikABB: Closed-form inverse kinematics of the ABB arm.
%
% q = ikABB(T06, dims)
% Returns every joint-angle solution that reaches the pose T06.
%
% q = [6x8] one solution per column, [theta1; ...; theta6] wrapped to (-pi, pi].
%     Columns are ordered by (shoulder flip, elbow, wrist). An unreachable pose
%     returns a 6x0 matrix. At a wrist singularity (sin(theta5) = 0) theta4 is set
%     to 0, so the two wrist columns of each arm configuration are identical.
%
% T06 = [4x4] desired homogeneous pose of frame 6 in the base frame
% dims = [1x5] link parameters [d1, a2, a3, d4, d6]
%
% Arael Anaya
% 10920967
% MEGN544
% 10/03/2026
function q = ikABB(T06, dims)
    d1 = dims(1);  a2 = dims(2);  a3 = dims(3);  d4 = dims(4);  d6 = dims(5);
    clampTol = 1e-9;
    sinTol = 1e-9;

    R06 = T06(1:3, 1:3);
    P06 = T06(1:3, 4);


    P05 = P06 - d6 * R06(:,3);

    r = sqrt(P05(1)^2 + P05(2)^2);
    D = sqrt(r^2 + (P05(3)-d1)^2);
    L = sqrt(a3^2 + d4^2);

    q = zeros(6, 0);
    if D < eps
        return;
    end

    cosBeta = (a2^2 + D^2 - L^2) / (2 * a2 * D);
    cosGamma = (a2^2 + L^2 - D^2) / (2 * a2 * L);

    if abs(cosBeta) > 1 + clampTol || abs(cosGamma) > 1 + clampTol
        return;
    end
    cosBeta = max(-1, min(1, cosBeta));
    cosGamma = max(-1, min(1, cosGamma));

    beta = atan2(sqrt(1 - cosBeta^2), cosBeta);
    gamma = atan2(sqrt(1 - cosGamma^2), cosGamma);
    gammaSupp = pi - gamma;
    delta = atan2(d4, a3);

    q = zeros(6, 8);
    k = 0;
    for flip = [0 1]
        thetaOne = atan2(P05(2), P05(1)) + flip*pi;
        rSigned = (1 - 2*flip) * r;

        for elbow = [1 -1]
            thetaTwo = atan2(P05(3)-d1, rSigned) + elbow*beta;
            thetaThree = -elbow*gammaSupp - delta;

            % 0R3 = Rz(th1) Ry(-(th2+th3))
            phi = thetaTwo + thetaThree;
            c1 = cos(thetaOne);  s1 = sin(thetaOne);
            R03 = [c1 -s1 0; s1 c1 0; 0 0 1] * ...
                  [cos(phi) 0 -sin(phi); 0 1 0; sin(phi) 0 cos(phi)];
            R36 = R03' * R06;

            for wrist = [1 -1]
                sinFive = wrist * sqrt(R36(1,3)^2 + R36(2,3)^2);
                thetaFive = atan2(sinFive, R36(3,3));

                if abs(sinFive) < sinTol
                    % Wrist singularity: only theta4 +/- theta6 is determined
                    thetaFour = 0;
                    if R36(3,3) > 0
                        thetaSix = atan2(R36(2,1), R36(1,1));
                    else
                        thetaSix = atan2(R36(2,1), -R36(1,1));
                    end
                else
                    thetaFour = atan2(-R36(2,3) / sinFive, -R36(1,3) / sinFive);
                    thetaSix = atan2(-R36(3,2) / sinFive, R36(3,1) / sinFive);
                end

                k = k + 1;
                qk = [thetaOne; thetaTwo; thetaThree; thetaFour; thetaFive; thetaSix];
                q(:,k) = atan2(sin(qk), cos(qk));
            end
        end
    end
end
