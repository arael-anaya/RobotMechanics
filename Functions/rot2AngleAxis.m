% rot2AngleAxis: Angle-times-axis (theta*k) from a rotation matrix.
%
% Omega = rot2AngleAxis(R)
% Angle-times-axis (theta*k) from a rotation matrix.
%
% Omega = [3x1] theta*k (rad)
%
% R = [3x3] rotation matrix
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function Omega = rot2AngleAxis(R)
    tol = 1e-8;

    % Vector from the skew-symmetric part of R: 2*sin(theta)*k
    v = [R(3,2) - R(2,3);
         R(1,3) - R(3,1);
         R(2,1) - R(1,2)];
    s = norm(v) / 2;            % sin(theta)
    c = (trace(R) - 1) / 2;     % cos(theta)
    theta = atan2(s, c);        % in [0, pi]

    if s > tol
        % General case
        k = v / (2 * s);
    elseif c > 0
        % theta = 0: axis is arbitrary, no rotation
        Omega = zeros(3,1);
        return;
    else
        B = (R + eye(3)) / 2;
        [~, i] = max(diag(B)); 
        k = B(:, i) / sqrt(B(i, i));
        k = k / norm(k);
    end

    Omega = theta * k;
end
