root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(fullfile(root, 'Functions'));
figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Latex', 'Figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end

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
[phi, theta, psi] = rot2ZYZ(R_i);   zyz_i = [phi(1); theta(1); psi(1)];
[phi, theta, psi] = rot2ZYZ(R_f);   zyz_f = [phi(1); theta(1); psi(1)];
Ra = cell(1, numel(s));  pa = zeros(3, numel(s));
for k = 1:numel(s)
    zyz = zyz_i + s(k) * (zyz_f - zyz_i);
    Ra{k} = rotZ(zyz(1)) * rotY(zyz(2)) * rotZ(zyz(3));
    pa(:,k) = p_i + s(k) * (p_f - p_i);
end
plotFrames(Ra, pa, '(a) ZYZ angles + position', fullfile(figDir, 'p4a.png'));

%% (b) Angle-axis + position
om_i = rot2AngleAxis(R_i);
om_f = rot2AngleAxis(R_f);

Rb = cell(1, numel(s));  pb = zeros(3, numel(s));
for k = 1:numel(s)
    om = om_i + s(k) * (om_f-om_i);
    Rb{k} = angleAxis2Rot(om);   % TODO
    pb(:,k) = p_i + s(k) * (p_f - p_i);
end
plotFrames(Rb, pb, '(b) Angle-Axis + position', fullfile(figDir, 'p4b.png'));

%% (c) Quaternion + position
% TODO: interpolate quaternions (rot2Quat / quat2Rot), enforce unit length
%       BEFORE converting each intermediate quaternion to a rotation
Rc = cell(1, numel(s));  pc = zeros(3, numel(s));
Q_i = rot2Quat(R_i);
Q_f = rot2Quat(R_f);

if dot(Q_i , Q_f) < 0
    Q_f = -Q_f;
end

for k = 1:numel(s)
    Q = Q_i +  s(k) * (Q_f-Q_i);
    Q = Q / norm(Q);
    Rc{k} =  quat2Rot(Q);
    pc(:,k) = p_i + s(k) * (p_f - p_i);
end
plotFrames(Rc, pc, '(c) Quaternion + position', fullfile(figDir, 'p4c.png'));

%% (d) Twist
% TODO: 
x_i = transform2Twist(T_f);
Rd = cell(1, numel(s));  pd = zeros(3, numel(s));
for k = 1:numel(s)
    x = [x_i(1:3) ; s(k)* x_i(4:6)];
    T = twist2Transform(x);
    Rd{k} = T(1:3,1:3);
    pd(:,k) = T(1:3, 4); 
end
plotFrames(Rd, pd, '(d) Twist', fullfile(figDir, 'p4d.png'));

%% (e) Comments
% TODO: comment on similarities / differences

function plotFrames(Rs, ps, ttl, file)
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
    exportgraphics(gcf, file, 'Resolution', 200);
end
