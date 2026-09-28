% multiplyQuat: Quaternion product.
%
% Q = multiplyQuat(Q_left, Q_right)
% Quaternion product.
%
% Q = [4x1] product quaternion [q0;q_vec]
%
% Q_left = [4x1] left quaternion [q0;q_vec]
% Q_right = [4x1] right quaternion [q0;q_vec]
%
% Arael Anaya
% 10920967
% MEGN544
% 09/19/2026
function Q = multiplyQuat(Q_left, Q_right)

    q0_left = Q_left(1);
    q_left = Q_left(2:end);
    q0_right = Q_right(1);
    q_right = Q_right(2:end);

    Q = [q0_left * q0_right - transpose(q_right) * q_left, 
            q0_left*q_right + q0_right*q_left + skew(q_left)*q_right];

end
