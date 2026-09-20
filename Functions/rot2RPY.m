% rot2RPY: Roll-pitch-yaw angles from a rotation matrix.
%
% [roll, pitch, yaw] = rot2RPY(R)
% Roll-pitch-yaw angles from a rotation matrix.
%
% roll = [2x1] roll solutions, +sqrt on top (rad)
% pitch = [2x1] pitch solutions, +sqrt on top (rad)
% yaw = [2x1] yaw solutions, +sqrt on top (rad); [0;0] if degenerate
%
% R = [3x3] rotation matrix
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function rot2RPY(R)
    % cp = sqrt(R(1,1)^2 + R(2,1)^2);
    % if cp < 1e-9
    
    %     pitch = atan2(-R(3,1), 0) * [1; 1];
    %     roll = atan2(-R(2,3), R(2,2)) * [1; 1];
    %     yaw = [0; 0];
    % else
    %     cps = [cp; -cp];
    %     pitch = atan2(-R(3,1), cps);
    %     roll = atan2(R(3,2) ./ cps, R(3,3) ./ cps);
    %     yaw = atan2(R(2,1) ./ cps, R(1,1) ./ cps);
    % end
end
