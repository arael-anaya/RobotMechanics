% constAccelInterp: Constant-acceleration (parabolic blend) interpolation
% through a set of trajectory points.
%
% [p, v, a] = constAccelInterp(t, trajectory, transPercent)
% Interpolates linearly between the trajectory points, blending between
% segments with constant-acceleration (parabolic) transitions, and returns
% the position, velocity, and acceleration at time t.
%
% p = position at time t
% v = velocity at time t
% a = acceleration at time t
%
% t = time at which to evaluate the trajectory
% trajectory = trajectory points (times and positions) to interpolate
% transPercent = percentage of each segment spent in the blend transition
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function [p, v, a] = constAccelInterp(t, trajectory, transPercent)

    
    

end
