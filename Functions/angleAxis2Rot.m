function R = angleAxis2Rot(omega)
    theta = norm(omega);
    k = (1/theta) * omega;
    R = cos(theta) * eye(3) + sin(theta) * skew(k) + (1 - cos(theta)) * (k * k');
end
