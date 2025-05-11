function Q = discrete_Q(F, G, dt, Qc)

    Z = zeros(size(F));
    [n, ~] = size(F);
    M = [-F G * Qc * G'; Z F'];
    L = expm(M * dt);
    L12 = L(1:n, n + 1:2 * n);
    L22 = L(n + 1:2 * n, n + 1:2 * n);
    Q = L22' * L12;
