% cpMap: Cross product matrix, w x v = X*v.
%
% X = cpMap(w)
% Cross product matrix, w x v = X*v.
%
% X = [3x3] skew-symmetric matrix
%
% w = [3x1] vector
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function X = cpMap(w)
    w = w(:);
    X = [    0,   -w(3),  w(2);
           w(3),    0,   -w(1);
          -w(2),   w(1),   0];
end
