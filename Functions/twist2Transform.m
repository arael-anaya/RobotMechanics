function T = twist2Transform(t)
    v = t(1:3);
    omega = t(4:6);

    theta = norm(omega);
    k = (1/theta) * omega;

    R = angleAxis2Rot(omega);
    d = (eye(3) - R) * skew(k) * v + omega * k' * v;

    T = [R, d; 0 0 0 1];
end
