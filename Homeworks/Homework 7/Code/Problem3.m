clear; clc;

addpath(genpath('../../../Functions'));

%% Joint states and DH parameters
theta    = [pi/3; pi/6; pi/6];
theta_d  = [0.5; -1.0; 1.5];
theta_dd = [0; 0; 0];           

a       = [0; 0.5; 0.3];
d       = [1; 0; 0];
alphaDH = [pi/2; 0; 0];

n = numel(a);

%% Frame origins (3 x 4) and z axes (3 x 4), expressed in frame 0
[o, z] = frameData(theta, a, d, alphaDH);

%% Velocity and acceleration recursion (column 1 = stationary base)
% odot is origin
w     = zeros(3, n+1); 
odot  = zeros(3, n+1);  
alpha = zeros(3, n+1);   
oddot = zeros(3, n+1);   

for i = 1:n
    r = o(:, i+1) - o(:, i);
    w(:, i+1)     = w(:, i) + theta_d(i) * z(:, i);
    alpha(:, i+1) = alpha(:, i) + theta_dd(i) * z(:, i) ...
                    + theta_d(i) * cross(w(:, i), z(:, i));
    odot(:, i+1)  = odot(:, i) + cross(w(:, i+1), r);
    oddot(:, i+1) = oddot(:, i) + cross(alpha(:, i+1), r) ...
                    + cross(w(:, i+1), cross(w(:, i+1), r));
end

%% Geometric Jacobian in frame 0
J = zeros(6, n);
for i = 1:n
    J(1:3, i) = cross(z(:, i), o(:, end) - o(:, i));
    J(4:6, i) = z(:, i);
end

%% Results (columns 2..4 are frames 1..3)
disp('w ='); disp(w(:, 2:end));
disp('odot ='); disp(odot(:, 2:end));
disp('alpha ='); disp(alpha(:, 2:end));
disp('oddot ='); disp(oddot(:, 2:end));
disp('J ='); disp(J);
Jtd = J * theta_d;
fprintf('J*theta_d (compare to the problem''s 6-vector):\n'); disp(round(Jtd, 4));

%% Checks
tol = 1e-9;
assert(norm(odot(:, end) - Jtd(1:3)) < tol, 'odot(end) != top half of J*theta_d');
assert(norm(w(:, end)    - Jtd(4:6)) < tol, 'w(end) != bottom half of J*theta_d');

dt = 1e-3;
p = @(t) endOrigin(theta + theta_d*t + 0.5*theta_dd*t^2, a, d, alphaDH);
v_fd   = (p(dt) - p(-dt)) / (2*dt);
acc_fd = (p(dt) - 2*p(0) + p(-dt)) / dt^2;
fprintf('velocity     error: %.3e\n', norm(v_fd   - odot(:, end)));
fprintf('acceleration error: %.3e\n', norm(acc_fd - oddot(:, end)));
assert(norm(v_fd   - odot(:, end))  < 1e-5, 'velocity finite-difference check failed');
assert(norm(acc_fd - oddot(:, end)) < 1e-3, 'acceleration finite-difference check failed');
disp('All checks passed.');

%% Local functions
function [o, z] = frameData(theta, a, d, alphaDH)
    n = numel(a);
    o = zeros(3, n+1);
    z = zeros(3, n+1);
    T = eye(4);
    z(:, 1) = T(1:3, 3);
    for i = 1:n
        T = T * dhTransform(a(i), d(i), alphaDH(i), theta(i));
        o(:, i+1) = T(1:3, 4);
        z(:, i+1) = T(1:3, 3);
    end
end

function p = endOrigin(theta, a, d, alphaDH)
    o = frameData(theta, a, d, alphaDH);
    p = o(:, end);
end
