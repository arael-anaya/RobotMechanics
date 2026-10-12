% plotTrajectory: Joint-space and Cartesian figures for one trajectory.
%
% plotTrajectory(t, q, P, label, figNum, saveDir, tag)
% Joint-space and Cartesian figures for one trajectory.
%
% t = [1 x m] times (s)
% q = [6 x m] joint values [theta1; theta2; d3; theta4; theta5; theta6]
% P = [3 x m] end-effector positions (m)
% label = descriptive name used in the figure titles
% figNum = figure number for the joint figure (figNum+1 is the Cartesian one)
% saveDir = folder to save PNGs into
% tag = short file-name tag
%
% Arael Anaya
% 10920967
% MEGN544
% 10/12/2026
function plotTrajectory(t, q, P, label, figNum, saveDir, tag)
    styles = {'-','--',':','-.','-o'};
    names = {'\theta_1','\theta_2','\theta_4','\theta_5','\theta_6'};
    rows = [1 2 4 5 6];

    f1 = figure(figNum); clf;
    subplot(2,1,1); hold on;
    for i = 1:5
        plot(t, q(rows(i),:), styles{i}, 'LineWidth', 1.5, 'MarkerIndices', 1:5:numel(t));
    end
    hold off; grid on;
    legend(names, 'Location', 'best');
    xlabel('Time (s)'); ylabel('Joint angle (rad)');
    title(['Joint Angles vs Time: ' label]);
    subplot(2,1,2);
    plot(t, q(3,:), '-', 'LineWidth', 1.5); grid on;
    legend('d_3', 'Location', 'best');
    xlabel('Time (s)'); ylabel('Prismatic joint d_3 (m)');
    title(['Prismatic Joint vs Time: ' label]);
    saveas(f1, fullfile(saveDir, ['P1_joint_' tag '.png']));

    f2 = figure(figNum+1); clf;
    plot3(P(1,:), P(2,:), P(3,:), '-', 'LineWidth', 1.5); hold on;
    plot3(P(1,1), P(2,1), P(3,1), 'go', 'MarkerFaceColor', 'g');
    plot3(P(1,end), P(2,end), P(3,end), 'rs', 'MarkerFaceColor', 'r');
    hold off; grid on; axis equal;
    legend('Path', 'Start', 'End', 'Location', 'best');
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title(['End-Effector Cartesian Path: ' label]);
    saveas(f2, fullfile(saveDir, ['P1_cartesian_' tag '.png']));
end
