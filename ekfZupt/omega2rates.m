function F = omega2rates(psi_nb)
    roll = psi_nb(1);
    pitch = psi_nb(2);
    singular_tol = 1e-4;

    if (pitch < 0)
        singular_check = abs(pitch + pi / 2);
    else
        singular_check = abs(pitch - pi / 2);
    end

    if (singular_check < singular_tol)
        error('Euler angle singularity at pitch = 90 degrees encountered.');
    end

    F = zeros(3, 3);

    F(1, 1) = 1; F(1, 2) = sin(roll) * tan(pitch); F(1, 3) = cos(roll) * tan(pitch);
    F(2, 1) = 0; F(2, 2) = cos(roll); F(2, 3) = -sin(roll);
    F(3, 1) = 0; F(3, 2) = sin(roll) / cos(pitch); F(3, 3) = cos(roll) / cos(pitch);
