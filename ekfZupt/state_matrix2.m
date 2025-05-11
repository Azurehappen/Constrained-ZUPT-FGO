function [Fc, Gc] = state_matrix2(q1, u1, q2, u2, Ts)

    [Fc1, Gc1] = state_matrix1(q1, u1);
    [Fc2, Gc2] = state_matrix1(q2, u2);
    N = size(Fc1, 1);
    M = size(Gc1, 2);
    Fc = [Fc1 zeros(N, N);
          zeros(N, N) Fc2];
    Gc = [Gc1 zeros(N, M);
          zeros(N, M) Gc2];
end
