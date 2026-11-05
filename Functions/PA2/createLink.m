% createLink: Package a robot link's DH and dynamic parameters into one row.
%
% L = createLink(a, d, alpha, theta, offset, centOfMass, mass, inertia)
% Builds a link description from its Denavit-Hartenberg parameters and its
% mass properties. Leave theta (or d) empty to mark that joint variable.
%
% L = link parameter vector [a, d, alpha, theta, offset, centOfMass, mass, inertia]
%
% a = link length (m)
% d = link offset (m)
% alpha = link twist (rad)
% theta = joint angle (rad)
% offset = joint variable offset
% centOfMass = center of mass of the link
% mass = mass of the link (kg)
% inertia = inertia of the link
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function L = createLink(a, d, alpha, theta, offset, centOfMass, mass, inertia)
    if nargin < 6, centOfMass = []; end
    if nargin < 7, mass = 0; end
    if nargin < 8, inertia = []; end
    L.a = a;
    L.d = d;
    L.alpha = alpha;
    L.offset = offset;
    L.centOfMass = centOfMass;
    L.theta = theta;
    L.mass = mass;
    L.inertia = inertia;
    
    if isempty(theta), L.isRotary = 1;
    elseif isempty(d), L.isRotary = 0;
    else,              L.isRotary = -1; %static link
    end
end
