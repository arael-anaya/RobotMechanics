% Problem1: Inverse kinematics of the ABB arm, tested over random poses
%
% Arael Anaya
% 10920967
% MEGN544
% 10/03/2026

clear; close all; clc;

addpath(genpath('../../../Functions'));

rng(544);
numTests = 100;
tol = 1e-8;
distTol = 1e-6;

d1 = 0.290;  a2 = 0.270;  a3 = 0.070;  d4 = 0.302;  d6 = 0.072;
dims = [d1 a2 a3 d4 d6];

DH = [      0     d1    pi/2      0   ;
            a2    0        0       0   ;
            a3    0    -pi/2      0   ;
            0     d4    pi/2      0   ;
            0     0    -pi/2      0   ;
            0     d6      0      0  ];

numPoseOK = 0;
numRecovered = 0;
numDistinct = 0;
worstPosErr = 0;
worstRotErr = 0;
minPairDist = inf;
testPosErr = zeros(numTests, 1);
testRotErr = zeros(numTests, 1);

for test = 1:numTests

    % theta5 is kept away from 0 and +/-pi, with a random sign so both wrist branches are exercised
    sign5 = 2*(rand > 0.5) - 1;
    qTrue = [ (2*rand-1)*pi ; (2*rand-1)*pi ; (2*rand-1)*pi ; ...
              (2*rand-1)*pi ; sign5*(rand*0.9+0.05)*pi ; (2*rand-1)*pi ];
    DH(:,4) = qTrue;

    Tfk = fkChain(DH);
    T06 = Tfk(:,:,end);

    solutions = ikABB(T06, dims);

    poseOK = size(solutions, 2) == 8;
    recovered = false;
    for k = 1:size(solutions, 2)
        DH(:,4) = solutions(:,k);
        Tk = fkChain(DH);
        posErr = norm(Tk(1:3,4,end) - T06(1:3,4));
        rotErr = norm(Tk(1:3,1:3,end) - T06(1:3,1:3), 'fro');
        worstPosErr = max(worstPosErr, posErr);
        worstRotErr = max(worstRotErr, rotErr);
        testPosErr(test) = max(testPosErr(test), posErr);
        testRotErr(test) = max(testRotErr(test), rotErr);
        if posErr > tol || rotErr > tol
            poseOK = false;
        end

        angErr = atan2(sin(solutions(:,k) - qTrue), cos(solutions(:,k) - qTrue));
        if norm(angErr) < 1e-6
            recovered = true;
        end
    end

    % Smallest wrapped-angle distance between any two of the solutions
    testMinDist = inf;
    for i = 1:size(solutions, 2)-1
        for j = i+1:size(solutions, 2)
            diff = solutions(:,i) - solutions(:,j);
            testMinDist = min(testMinDist, norm(atan2(sin(diff), cos(diff))));
        end
    end
    minPairDist = min(minPairDist, testMinDist);

    numPoseOK = numPoseOK + poseOK;
    numRecovered = numRecovered + recovered;
    numDistinct = numDistinct + (size(solutions, 2) == 8 && testMinDist > distTol);
end

% Wrist-singular test case (theta5 = 0): every returned solution must still reach the pose
qSing = [0.4; 0.3; -0.5; 0.7; 0; -0.2];
DH(:,4) = qSing;
Tfk = fkChain(DH);
T06 = Tfk(:,:,end);
solutions = ikABB(T06, dims);
singErr = 0;
for k = 1:size(solutions, 2)
    DH(:,4) = solutions(:,k);
    Tk = fkChain(DH);
    singErr = max(singErr, norm(Tk(:,:,end) - T06, 'fro'));
end
singOK = size(solutions, 2) == 8 && singErr <= tol;

fprintf('Random test cases:                         %d\n', numTests);
fprintf('All 8 solutions reproduce target pose:     %d / %d\n', numPoseOK, numTests);
fprintf('True joint angles found among solutions:   %d / %d\n', numRecovered, numTests);
fprintf('8 distinct solutions (min dist > %.0e):   %d / %d\n', distTol, numDistinct, numTests);
fprintf('Smallest pairwise solution distance:       %.3e rad\n', minPairDist);
fprintf('Worst position error:                      %.3e\n', worstPosErr);
fprintf('Worst rotation error (Frobenius norm):     %.3e\n', worstRotErr);
fprintf('Singular case (theta5 = 0) pose error:     %.3e\n', singErr);

fig = figure('Color', 'w');
semilogy(1:numTests, max(testPosErr, eps), 'o', 'MarkerFaceColor', [0.1 0.3 0.7], 'MarkerSize', 4); hold on;
semilogy(1:numTests, max(testRotErr, eps), 's', 'MarkerFaceColor', [0.85 0.4 0.1], 'MarkerSize', 4);
yline(tol, '--r', 'Tolerance');
ylim([1e-16 1e-6]);
xlabel('Test case'); ylabel('Worst error over 8 solutions');
legend('Position error (m)', 'Rotation error (Frobenius)', 'Location', 'best');
title('Problem 1: IK verification over 100 random test cases');
grid on;
exportgraphics(fig, '../Latex/Figures/Problem1ErrorFigure.png', 'Resolution', 200);

fid = fopen('../Latex/Problem1Results.tex', 'w');
fprintf(fid, '\\begin{tabular}{l r}\n\\hline\n');
fprintf(fid, 'Random test cases & %d \\\\\n', numTests);
fprintf(fid, 'All 8 solutions reproduce target pose & %d / %d \\\\\n', numPoseOK, numTests);
fprintf(fid, 'True joint angles found among solutions & %d / %d \\\\\n', numRecovered, numTests);
fprintf(fid, '8 distinct solutions (min pairwise distance $>10^{-6}$ rad) & %d / %d \\\\\n', numDistinct, numTests);
fprintf(fid, 'Smallest pairwise solution distance (rad) & $%.2e$ \\\\\n', minPairDist);
fprintf(fid, 'Worst position error (m) & $%.2e$ \\\\\n', worstPosErr);
fprintf(fid, 'Worst rotation error (Frobenius norm) & $%.2e$ \\\\\n', worstRotErr);
fprintf(fid, 'Singular case ($\\theta_5=0$), all solutions reach pose & %s \\\\\n', string(singOK));
fprintf(fid, '\\hline\n\\end{tabular}\n');
fclose(fid);
