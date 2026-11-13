clear; clc;

%% Symbols and DH table (a, d, alpha fixed; theta is the joint variable)
syms th1 th2 th3 L1 L2 L3 real
th = [th1; th2; th3];
a       = [0;  L2; L3];
d       = [L1; 0;  0];
alphaDH = sym([pi/2; 0; 0]);   % exact symbolic so cos(pi/2) = 0

%% (a) Forward kinematics: 0T3 = A1*A2*A3
T = eye(4);
Ts = cell(1, 3);
for i = 1:3
    ct = cos(th(i)); st = sin(th(i));
    ca = cos(alphaDH(i)); sa = sin(alphaDH(i));
    A = [ct, -st*ca,  st*sa, a(i)*ct;
         st,  ct*ca, -ct*sa, a(i)*st;
         0,   sa,     ca,    d(i);
         0,   0,      0,     1];
    T = simplify(T * A);
    Ts{i} = T;                       % Ts{i} = 0T_i
end
p = simplify(T(1:3, 4));
disp('(a) Tool position 0p:'); disp(p);

%% (b) Differentiate the position equations
Jv_diff = simplify(jacobian(p, th));
disp('(b) Jv by differentiation:'); disp(Jv_diff);

%% (c) Geometric method: column k = z_{k-1} x d_{k-1,N}
z0 = sym([0; 0; 1]);
o  = {sym([0; 0; 0]), Ts{1}(1:3, 4), Ts{2}(1:3, 4)};   % o_0, o_1, o_2
z  = {z0, Ts{1}(1:3, 3), Ts{2}(1:3, 3)};               % z_0, z_1, z_2
Jv_geo = sym(zeros(3, 3));
for k = 1:3
    dk = p - o{k};                                      % 0d_{k-1,N}
    Jv_geo(:, k) = simplify(cross(z{k}, dk));
    fprintf('z_%d = ', k-1); disp(simplify(z{k}.'));
    fprintf('d_%d,3 = ', k-1); disp(simplify(dk.'));
end
disp('(c) Jv by z x d:'); disp(Jv_geo);
disp('Symbolic difference (b)-(c):'); disp(simplify(Jv_diff - Jv_geo));

%% (d) Numeric evaluation
vals = {L1, L2, L3, th1, th2, th3};
nums = {1, 0.5, 0.3, 0, pi/4, pi/4};
Jb = double(subs(Jv_diff, vals, nums));
Jc = double(subs(Jv_geo,  vals, nums));
pn = double(subs(p, vals, nums));
disp('(d) 0p ='); disp(pn.');
disp('(d) Jv from (b) ='); disp(Jb);
disp('(d) Jv from (c) ='); disp(Jc);
fprintf('max |(b)-(c)| = %g\n', max(abs(Jb - Jc), [], 'all'));
