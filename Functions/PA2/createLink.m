% createLink: Package a robot link's DH and dynamic parameters in a struct.
%
% L = createLink(a, d, alpha, theta, offset, com, mass, inertia)
% Builds a link description from its Denavit-Hartenberg parameters and its
% mass properties. Pass an empty array ([]) for the joint variable that
% changes: theta for a rotary joint, or d for a prismatic joint. If both
% theta and d are given the link is static.
%
% L = struct with the members a, d, alpha, theta, offset, centOfMass, com
%     (a copy of centOfMass), mass, inertia, and isRotary. isRotary is 1 if
%     the joint is rotary, 0 if it is prismatic, and -1 if the link is static
%
% a = DH parameter a, link length (m)
% d = DH parameter d, link offset (m), [] if the joint is prismatic
% alpha = DH parameter alpha, link twist (rad)
% theta = joint angle from the encoder (rad), [] if the joint is rotary
% offset = radians (or meters for prismatic) between the encoder zero and
%          the DH zero, thetaDH = Theta - offset
% centOfMass = 3x1 position of the link center of mass in the link frame (m)
% mass = mass of the link (kg)
% inertia = 3x3 moment of inertia of the link in the link frame (kg m^2)
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
    L.centOfMass = centOfMass;
    L.com = centOfMass;
    L.theta = theta;
    L.mass = mass;
    L.inertia = inertia;
    
    if isempty(theta), L.isRotary = 1;
    elseif isempty(d), L.isRotary = 0;
    else,              L.isRotary = -1; %static link
    end
end
