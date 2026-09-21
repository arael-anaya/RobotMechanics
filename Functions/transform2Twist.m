% transform2Twist: Twist vector from a homogeneous transform.
%
% t = transform2Twist(H)
% Twist vector from a homogeneous transform.
%
% t = [6x1] twist stacked [v;w th]
%
% H = [4x4] homogeneous transform
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function t = transform2Twist(H)
    R = H(1:3,1:3);
    d = H(1:3,4);

    omega = rot2AngleAxis(R);
    theta = norm(omega);
    if theta < 1e-12
        t = [d; zeros(3,1)];
        return;
    end
    k = (1/theta) * omega;

    v = ((sin(theta) / (2*(1-cos(theta)))) * eye(3) + (2*(1-cos(theta)) -theta*sin(theta))/(2 * theta*(1-cos(theta))) * k * transpose(k) - .5 * skew(k)) * d;

    t = [v ; omega];

% TODO: implement
end
