root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(fullfile(root, 'Functions'));


% Twists are xi = [v; omega]
xi_a = [3/pi; 0; 0; pi/3; 0; 0];
xi_b = [0; 0.5; 0; 0; pi/2; 0];
xi_c = [1; 0; 0; 0; 0; pi/3];

T_a = twist2Transform(xi_a);
T_b = twist2Transform(xi_b);
T_c = twist2Transform(xi_c);

disp('(a) T ='); disp(T_a);
disp('(b) T ='); disp(T_b);
disp('(c) T ='); disp(T_c);
