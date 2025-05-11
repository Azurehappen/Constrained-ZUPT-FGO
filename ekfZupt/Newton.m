function [lam] = Newton(tol, E, Alpha, dist)
    %UNTITLED5 Summary of this function goes here
    %   Detailed explanation goes here
    flag = 1;

    while flag

        lam0 = 0.25;
        Err = 1;

        while Err > tol

            flam = sum((Alpha .* E .^ 2) ./ (1 + lam0 * Alpha) .^ 2) - dist ^ 2;
            flamDot = -2 * sum(((Alpha .^ 2) .* (E .^ 2)) ./ ((1 + lam0 * Alpha) .^ 3));

            lam = lam0 - flam / flamDot;
            Err = abs(lam - lam0);
            lam0 = lam;

        end

        if lam > 0
            flag = 0;
        end

    end

end
