% constAccelInterp: Constant-acceleration (parabolic blend) interpolation
% through a set of trajectory points.
%
% [p, v, a] = constAccelInterp(t, trajectory, transPercent)
% Interpolates linearly between the trajectory points, blending between
% segments with constant-acceleration (parabolic) transitions, and returns
% the position, velocity, and acceleration at time t.
%
% Before the first waypoint time or after the last, the first or last point
% is returned with zero velocity and acceleration. The first and last
% waypoints are not blended.
%
% p = position at time t, 1xM vector
% v = velocity at time t, 1xM vector
% a = acceleration at time t, 1xM vector
%
% t = time at which to evaluate the trajectory (scalar, seconds)
% trajectory = Nx(M+1) array of N waypoints of dimension M. The first column
%              is the time of each waypoint (increasing) and the remaining M
%              columns are the point to be reached at that time
% transPercent = half-width of each blend as a fraction of the shorter of the
%                two adjacent segment durations, in the range [0, 0.5]. The
%                full blend lasts twice this and is centered on the waypoint
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function [p, v, a] = constAccelInterp(t, trajectory, transPercent)

    times = trajectory(:,1);
    points = trajectory(:,2:end);
    N = size(trajectory,1);
    M = size(points,2);
    if t <= times(1)
        p = points(1,:);
        a = zeros(1, M);
        v = zeros(1, M);
        return
    elseif t >= times(end)
        p = points(end,:);
        a = zeros(1, M);
        v = zeros(1, M);
        return
    end

    k = find(t >= times, 1, 'last');
    segmentVel = diff(points) ./ diff(times);
    h = zeros(N,1);
    for j = 2:N-1
        h(j) = transPercent * min(times(j) - times(j-1) , times(j+1) - times(j));
    end

    if k > 1 && h(k) > 0 && t < times(k) + h(k)
        w = k;
        v1 = segmentVel(k-1,:);
        v2 = segmentVel(k,:);
        hw = h(k);
        isBlend = true;
    elseif k+1 < N && h(k+1) > 0 && t > times(k+1) - h(k+1)
        w = k+1;
        v1 = segmentVel(k,:);
        v2 = segmentVel(k+1,:);
        hw = h(k+1);
        isBlend = true;
    else
        isBlend = false;
    end


    if isBlend
        tau = t - (times(w) - hw);
        a = (v2-v1) / (2*hw);
        v = v1 + a*tau;

        p_start = points(w,:) - v1*hw;
        p = p_start + v1 * tau + .5 * a * tau^2;
    else
        v = segmentVel(k,:);
        a = zeros(1,M);
        p = points(k,:) + v*(t-times(k));
    end




end
