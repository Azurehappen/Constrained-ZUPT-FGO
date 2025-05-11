function [y, q] = Navigation_equations(x, u, q, Ts)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  This function takes as input x - state vector, u - imu vector and
    %  q - quaternion, returns updated state vector - y and updated
    %  quaternion - q
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    y = zeros(size(x));

    w_tb = u(4:6); % angular veolocity, body frame with respect to inertial frame, resolved in body frame

    % euler angle rate
    Fr = omega2rates(x(7:9));
    y(7:9) = x(7:9) + Ts * (Fr * w_tb);

    P = w_tb(1) * Ts;
    Q = w_tb(2) * Ts;
    R = w_tb(3) * Ts;

    OMEGA = zeros(4);
    OMEGA(1, 1:4) = 0.5 * [0 R -Q P];
    OMEGA(2, 1:4) = 0.5 * [-R 0 P Q];
    OMEGA(3, 1:4) = 0.5 * [Q -P 0 R];
    OMEGA(4, 1:4) = 0.5 * [-P -Q -R 0];

    v = norm(w_tb) * Ts;

    if v ~= 0
        q = (cos(v / 2) * eye(4) + 2 / v * sin(v / 2) * OMEGA) * q;
        q = q ./ norm(q);
    end

    Rb2t = q2dcm(q);

    latitude = 44.975551204890714;
    altitude = 256;
    g = gravity(latitude, altitude);

    Cb2n = Rt2b(x(7:9))';

    g_t = [0 0 g]';
    f_t = q2dcm(q) * u(1:3);
    acc_t = f_t + g_t;

    A = eye(6);
    A(1, 4) = Ts;
    A(2, 5) = Ts;
    A(3, 6) = Ts;

    B = [(Ts ^ 2) / 2 * eye(3); Ts * eye(3)];
    y(1:6) = A * x(1:6) + B * acc_t;
    y(10:15) = x(10:15);
end
