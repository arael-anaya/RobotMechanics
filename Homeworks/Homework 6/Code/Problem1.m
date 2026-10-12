% Problem1: Trajectory planning for the 1P5R arm in joint, angle-axis,
% quaternion, and twist space
%
% Arael Anaya
% 10920967
% MEGN544
% 10/03/2026

clear; close all; clc;

addpath(genpath('../../../Functions'));

a1 = 1;
a2 = 0.5;
T06_0 = [ 1 0  0  1.00;
          0 0 -1  0.25;
          0 1  0  0.25;
          0 0  0  1   ];

T06_f = [ 0 -1  0  0.50;
          1  0  0 -0.50;
          0  0  1  0   ;
          0  0  0  1   ];

t = linspace(0,2,50);
saveDir = '../Latex/Figures';
playMovies = true;

p0 = T06_0(1:3,4);  R0 = T06_0(1:3,1:3);
pf = T06_f(1:3,4);  Rf = T06_f(1:3,1:3);
angRows = [1 2 4 5 6];      % angular joints (d3 is row 3)

% Joint space
q0 = ik_hw6(T06_0, a1, a2);
qf = ik_hw6(T06_f, a1, a2);
C = cubicCoeffs(q0, qf, 0, 2);
Q = cubicInterp(C, 0, t);
q_js = Q;

% Angle Axis
X0 = [p0; rot2AngleAxis(R0)];  Xf = [pf; rot2AngleAxis(Rf)];
Q_aa = cubicInterp(cubicCoeffs(X0, Xf, 0, 2), 0, t);
for k = 1:numel(t)
    T = [angleAxis2Rot(Q_aa(4:6,k)), Q_aa(1:3,k); 0 0 0 1];
    q_aa(:,k) = ik_hw6(T, a1, a2);
end

% Quaternion
Q0 = rot2Quat(R0);  Qf = rot2Quat(Rf);
if dot(Q0,Qf) < 0, Qf = -Qf; end
Q_q = cubicInterp(cubicCoeffs([p0; Q0], [pf; Qf], 0, 2), 0, t);
for k = 1:numel(t)
    quat = Q_q(4:7,k) / norm(Q_q(4:7,k));
    T = [quat2Rot(quat), Q_q(1:3,k); 0 0 0 1];
    q_q(:,k) = ik_hw6(T, a1, a2);
end

% Twist
xi = transform2Twist(inv(T06_0) * T06_f);
s = cubicInterp(cubicCoeffs(0, 1, 0, 2), 0, t);
for k = 1:numel(t)
    T = T06_0 * twist2Transform(s(k) * xi);
    q_tw(:,k) = ik_hw6(T, a1, a2);
end

% Unwrap angular joints so IK branch jumps of 2*pi do not appear
q_aa(angRows,:) = unwrap(q_aa(angRows,:), [], 2);
q_q(angRows,:)  = unwrap(q_q(angRows,:),  [], 2);
q_tw(angRows,:) = unwrap(q_tw(angRows,:), [], 2);

% Checks: no NaN, endpoints match, no jumps between samples
methods = {'Joint space', 'Angle-axis', 'Quaternion', 'Twist'};
hist = {q_js, q_aa, q_q, q_tw};
for i = 1:4
    qh = hist{i};
    fprintf('%-12s NaN: %d | start err %.2e | end err %.2e | max step %.3f\n', ...
        methods{i}, any(isnan(qh(:))), norm(qh(:,1)-q0), norm(qh(:,end)-qf), ...
        max(abs(diff(qh(angRows,:),1,2)),[],'all'));
end

% Cartesian positions from forward kinematics
P_js = armPositions(q_js, a1, a2);
P_aa = armPositions(q_aa, a1, a2);
P_q  = armPositions(q_q,  a1, a2);
P_tw = armPositions(q_tw, a1, a2);
fprintf('FK start err %.2e | end err %.2e\n', norm(P_js(:,1)-p0), norm(P_js(:,end)-pf));

% Figures
plotTrajectory(t, q_js, P_js, 'Joint-Space Interpolation',      1, saveDir, 'jointspace');
plotTrajectory(t, q_aa, P_aa, 'Cartesian/Angle-Axis Interpolation', 3, saveDir, 'angleaxis');
plotTrajectory(t, q_q,  P_q,  'Cartesian/Quaternion Interpolation', 5, saveDir, 'quaternion');
plotTrajectory(t, q_tw, P_tw, 'Twist-Space Interpolation',      7, saveDir, 'twist');

% Movies
if playMovies
    for i = 1:4
        qh = hist{i};
        figure(20+i); clf;
        for k = 1:numel(t)
            drawArm(qh(1,k), qh(2,k), qh(3,k), qh(4,k), qh(5,k), qh(6,k), 20+i, T06_f);
            title([methods{i} ' Interpolation, t = ' num2str(t(k),'%.2f') ' s']);
            drawnow; pause(0.05);
        end
    end
end
