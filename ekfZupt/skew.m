function S = skew(v_in)

    S = [0 -v_in(3) v_in(2); v_in(3) 0 -v_in(1); -v_in(2) v_in(1) 0];
