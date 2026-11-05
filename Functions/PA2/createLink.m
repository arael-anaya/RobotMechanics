% createLink: Package a robot link's DH and dynamic parameters into one row.
%
% L = createLink(a, d, alpha, theta, offset, com, mass, inertia)
% Builds a link description from its Denavit-Hartenberg parameters and its
% mass properties. Leave theta (or d) empty to mark that joint variable.
%
% L = link parameter vector [a, d, alpha, theta, offset, com, mass, inertia]
%
% a = link length (m)
% d = link offset (m)
% alpha = link twist (rad)
% theta = joint angle (rad)
% offset = joint variable offset
% com = center of mass of the link (3x1 vector in the link frame, m); stored in L.com
% mass = mass of the link (kg)
% inertia = inertia of the link (3x3 matrix about the center of mass, kg*m^2)
%
% Joint type is stored in L.isRotary and is determined by which of theta or d
% is left empty:
%   theta empty -> rotary (revolute) joint, L.isRotary = 1
%   d empty     -> prismatic joint,         L.isRotary = 0
%   neither     -> static (fixed) link,     L.isRotary = -1
%
% The returned struct L has the fields: a, d, alpha, offset, com, theta,
% mass, inertia, and isRotary. The optional arguments com, mass, and inertia
% default to [], 0, and [] respectively when they are not supplied.
%
% Example:
%   L = createLink(0.5, 0.1, pi/2, [], 0, [0.25;0;0], 2.0, eye(3)*0.01);
%   % L.isRotary is 1 because theta was left empty (rotary joint).
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function L = createLink(a, d, alpha, theta, offset, com, mass, inertia)
    if nargin < 6, com = []; end
    if nargin < 7, mass = 0; end
    if nargin < 8, inertia = []; end
    L.a = a;
    L.d = d;
    L.alpha = alpha;
    L.offset = offset;
    L.com = com;
    L.theta = theta;
    L.mass = mass;
    L.inertia = inertia;
    
    if isempty(theta), L.isRotary = 1;
    elseif isempty(d), L.isRotary = 0;
    else,              L.isRotary = -1; %static link
    end
end
