function [x_out, q_out] = comp_internal_states(x_in, dx, q_in)
    R = q2dcm(q_in);

    x_out = x_in + dx;

    epsilon = dx(7:9);
    OMEGA = [0 -epsilon(3) epsilon(2); epsilon(3) 0 -epsilon(1); -epsilon(2) epsilon(1) 0];
    R = (eye(3) - OMEGA) * R;

    x_out(7) = atan2(R(3, 2), R(3, 3));
    x_out(8) = -atan(R(3, 1) / sqrt(1 - R(3, 1) ^ 2));
    x_out(9) = atan2(R(2, 1), R(1, 1));

    q_out = dcm2q(R);

end
