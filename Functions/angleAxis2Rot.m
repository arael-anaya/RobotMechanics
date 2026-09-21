function R = angleAxis2Rot(omega)
    theta = norm(omega);
    if theta < 1e-12
        R = eye(3);
        return;
    end    
    k = (1/theta) * omega;
    R = cos(theta) * eye(3) + sin(theta) * skew(k) + (1 - cos(theta)) * (k * k');
end
