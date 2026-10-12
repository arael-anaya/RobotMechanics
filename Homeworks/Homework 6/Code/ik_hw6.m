% ik_hw6: Inverse kinematics for the 1P5R arm of HW 6.
%
% thetas = ik_hw6(T, a1, a2)
% Inverse kinematics for the 1P5R arm of HW 6.
%
% thetas = [6x1] joint values [theta1; theta2; d3; theta4; theta5; theta6]
%     (rad, with d3 in length units)
%
% T = [4x4] desired end-effector pose
% a1 = length of link 1
% a2 = length of link 2
%
% Arael Anaya
% 10920967
% MEGN544
% 10/12/2026
function thetas = ik_hw6(T, a1, a2)
    px = T(1,4);  py = T(2,4);  pz = T(3,4);
    R = T(1:3,1:3);

    theta2 = 2*atan(sqrt(((a1+a2)^2 - (px^2+py^2)) / ((px^2+py^2) - (a1-a2)^2)));
    theta1 = atan2(py,px) - atan2(a2*sin(theta2), a1 + a2*cos(theta2));
    d3 = pz;
    theta5 = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    if abs(sin(theta5)) < 1e-9
        theta5 = 0;
        theta6 = 0;
        theta4 = atan2(R(2,1), R(1,1)) - theta1 - theta2 - theta6;
    else
        theta4 = atan2(-R(2,3)/sin(theta5), -R(1,3)/sin(theta5)) - theta1 - theta2;
        theta6 = atan2(-R(3,2)/sin(theta5), R(3,1)/sin(theta5));
    end

    thetas = [theta1; theta2; d3; theta4; theta5; theta6];
end
