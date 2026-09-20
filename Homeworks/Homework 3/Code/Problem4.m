root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(fullfile(root, 'Functions'));

T_i = eye(4);
T_f = [ 0.5000 -0.6124  0.6124  0.8415;
        0.6124  0.7500  0.2500  0.3251;
       -0.6124  0.2500  0.7500 -0.3251;
        0       0       0       1     ];

R_i = T_i(1:3,1:3);  p_i = T_i(1:3,4);
R_f = T_f(1:3,1:3);  p_f = T_f(1:3,4);

N = 5;                          % intermediate points (minimum 5)
s = linspace(0, 1, N + 2);      % includes both endpoints

%% (a) ZYZ angles + position
% TODO: extract ZYZ angles from R_i and R_f (no helper exists for ZYZ)
zyz_i = [0; 0; 0];
zyz_f = [0; 0; 0];
Ra = cell(1, numel(s));  pa = zeros(3, numel(s));
for k = 1:numel(s)
    zyz = zyz_i + s(k) * (zyz_f - zyz_i);
    Ra{k} = rotZ(zyz(1)) * rotY(zyz(2)) * rotZ(zyz(3));
    pa(:,k) = p_i + s(k) * (p_f - p_i);
end
plotFrames(Ra, pa, '(a) ZYZ angles + position');

%% (b) Angle-axis + position
% TODO: interpolate the angle-axis vector (rot2AngleAxis / angleAxis2Rot)
Rb = cell(1, numel(s));  pb = zeros(3, numel(s));
for k = 1:numel(s)
    Rb{k} = R_i;   % TODO
    pb(:,k) = p_i + s(k) * (p_f - p_i);
end
plotFrames(Rb, pb, '(b) Angle-Axis + position');

%% (c) Quaternion + position
% TODO: interpolate quaternions (rot2Quat / quat2Rot), enforce unit length
%       BEFORE converting each intermediate quaternion to a rotation
Rc = cell(1, numel(s));  pc = zeros(3, numel(s));
for k = 1:numel(s)
    Rc{k} = R_i;   % TODO
    pc(:,k) = p_i + s(k) * (p_f - p_i);
end
plotFrames(Rc, pc, '(c) Quaternion + position');

%% (d) Twist
% TODO: xi = transform2Twist(T_f) and T(s) = twist2Transform(s * xi)
Rd = cell(1, numel(s));  pd = zeros(3, numel(s));
for k = 1:numel(s)
    Rd{k} = R_i;   % TODO
    pd(:,k) = p_i; % TODO
end
plotFrames(Rd, pd, '(d) Twist');

%% (e) Comments
% TODO: comment on similarities / differences

function plotFrames(Rs, ps, ttl)
    figure; hold on; grid on; axis equal;
    plot3(ps(1,:), ps(2,:), ps(3,:), 'k--');
    for k = 1:numel(Rs)
        R = Rs{k};  p = ps(:,k);
        quiver3(p(1), p(2), p(3), R(1,1), R(2,1), R(3,1), 0.3, 'r');
        quiver3(p(1), p(2), p(3), R(1,2), R(2,2), R(3,2), 0.3, 'g');
        quiver3(p(1), p(2), p(3), R(1,3), R(2,3), R(3,3), 0.3, 'b');
    end
    xlabel('x'); ylabel('y'); zlabel('z'); title(ttl);
    view(3); hold off;
end
