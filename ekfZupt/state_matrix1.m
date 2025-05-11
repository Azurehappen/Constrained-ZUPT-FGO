function [Fc, Gc] = state_matrix1(q, u)

    Lat = 44.975551204890714;
    omega_e = 7.292115e-5;
    omega_et = [0 -omega_e * cosd(Lat) -omega_e * sind(Lat)]';
    Omega_et = skew(omega_et);

    Rb2t = q2dcm(q);

    f_t = Rb2t * u(1:3);

    St = [0 -f_t(3) f_t(2); f_t(3) 0 -f_t(1); -f_t(2) f_t(1) 0];

    O = zeros(3);

    I = eye(3);

    Fc = [O I O O O;
          O O St Rb2t O;
          O O O O -Rb2t;
          O O O O O;
          O O O O O];

    Gc = [O O O O; Rb2t O O O; O -Rb2t O O; O O I O; O O O I];

end
