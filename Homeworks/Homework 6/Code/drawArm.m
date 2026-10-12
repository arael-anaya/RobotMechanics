% drawArm: Draw the HW 6 arm configuration in 3D space.
%
% drawArm(t1, t2, d3, t4, t5, t6, figureNum, TargetPose)
% Draw the HW 6 arm configuration in 3D space.
%
% t1, t2, d3, t4, t5, t6 = variable DH parameters for the arm
% figureNum = figure number to plot in; a new figure is made if not provided
% TargetPose = [4x4] optional target pose the arm is trying to reach
%
% Arael Anaya
% 10920967
% MEGN544
% 10/12/2026
function drawArm(t1,t2,d3,t4,t5,t6, figureNum, TargetPose)

if ~exist('figureNum','var')
    figureNum = figure();
end
a1 = 1;
a2 = .5;

p0 = [0;0;0];
p1 = p0 + [a1*cos(t1);a1*sin(t1);0];
p2 = p1 + [a2*cos(t1+t2);a2*sin(t1+t2);0];
p3 = p2 + [0;0;d3];

R = [cos(t1 + t2 + t4)*cos(t5)*cos(t6) - sin(t1 + t2 + t4)*sin(t6), - sin(t1 + t2 + t4)*cos(t6) - cos(t1 + t2 + t4)*cos(t5)*sin(t6), -cos(t1 + t2 + t4)*sin(t5);
    cos(t1 + t2 + t4)*sin(t6) + sin(t1 + t2 + t4)*cos(t5)*cos(t6),   cos(t1 + t2 + t4)*cos(t6) - sin(t1 + t2 + t4)*cos(t5)*sin(t6), -sin(t1 + t2 + t4)*sin(t5);
    cos(t6)*sin(t5),                                                -sin(t5)*sin(t6),                    cos(t5)];

L = [p0 p1 p2 p3]';

figure(figureNum)
plot3(L(:,1),L(:,2),L(:,3),'Color','k','LineWidth',2);
hold on;

if exist('TargetPose','var')
    quiver3(ones(3,1)*TargetPose(1,4),ones(3,1)*TargetPose(2,4),ones(3,1)*TargetPose(3,4),TargetPose(1,1:3)',TargetPose(2,1:3)',TargetPose(3,1:3)',.5,'r');
    plot3(TargetPose(1,4),TargetPose(2,4),TargetPose(3,4),'Marker','*','MarkerSize',8,'Color','r');
end

quiver3(ones(3,1)*p3(1),ones(3,1)*p3(2),ones(3,1)*p3(3),R(1,:)',R(2,:)',R(3,:)',.5,'b');

hold off;
axis([-1.5,1.5,-1.5,1.5,-1.5,1.5])


end