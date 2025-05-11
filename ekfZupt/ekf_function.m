function [] = ekf_function(posTrigger, stepTrigger, fileName)
    %%-------------------------------------------------------------------------
    % u: IMU data vector (6 x 1)
    % y: position measurement
    %%-------------------------------------------------------------------------
    close all;

    % load noise statistics and initial alignment parameters
    run('constants.m');
    recordLen = 2 * 60 * 60;

    ind1 = 1536; ind2 = 5027; ind3 = 5028;
    yawFoot = 0/180 * pi;
    yawLeft = 0/180 * pi;

    % load IMU measurements
    load("ImuFootLeft.mat");
    load("posFoot.mat");
    load("posLeft.mat");

    imuMeasDataFoot = ImuFootLeft(:, 1:6);
    imuMeasDataLeft = ImuFootLeft(:, 7:12);

    % rearrange the axes of left and right foot imu
    imuMeasDataFoot = [imuMeasDataFoot(:, 1) -imuMeasDataFoot(:, 2) ...
                           -imuMeasDataFoot(:, 3) imuMeasDataFoot(:, 4) -imuMeasDataFoot(:, 5) ...
                           -imuMeasDataFoot(:, 6)];
    imuMeasDataLeft = [imuMeasDataLeft(:, 1) -imuMeasDataLeft(:, 2) ...
                           -imuMeasDataLeft(:, 3) imuMeasDataLeft(:, 4) -imuMeasDataLeft(:, 5) ...
                           -imuMeasDataLeft(:, 6)];

    % reset the initial alignment by leveling technique
    accelXFootStationary = mean(imuMeasDataFoot(ind1:ind2, 1));
    accelYFootStationary = mean(imuMeasDataFoot(ind1:ind2, 2));
    accelZFootStationary = mean(imuMeasDataFoot(ind1:ind2, 3));
    initialPitchFoot = atan(accelXFootStationary / sqrt(accelYFootStationary ^ 2 ...
        + accelZFootStationary ^ 2));
    initialRollFoot = atan2(-accelYFootStationary, -accelZFootStationary);
    initialYawFoot = yawFoot;

    accelXLeftStationary = mean(imuMeasDataLeft(ind1:ind2, 1));
    accelYLeftStationary = mean(imuMeasDataLeft(ind1:ind2, 2));
    accelZLeftStationary = mean(imuMeasDataLeft(ind1:ind2, 3));
    initialPitchLeft = atan(accelXLeftStationary / sqrt(accelYLeftStationary ^ 2 ...
        + accelZLeftStationary ^ 2));
    initialRollLeft = atan2(-accelYLeftStationary, -accelZLeftStationary);
    initialYawLeft = yawLeft;

    initialGyroBiasXFoot = -0.009;
    initialGyroBiasYFoot = 0.0077;
    initialGyroBiasZFoot = -0.001;
    initialGyroBiasFoot = [initialGyroBiasXFoot;
                       initialGyroBiasYFoot;
                       initialGyroBiasZFoot];

    initialGyroBiasXLeft = 0.005;
    initialGyroBiasYLeft = 0.003;
    initialGyroBiasZLeft = -0.003;
    initialGyroBiasLeft = [initialGyroBiasXLeft;
                       initialGyroBiasYLeft;
                       initialGyroBiasZLeft];

    imuMeasFoot = imuMeasDataFoot(ind3:ind3 + recordLen, :)';
    imuMeasLeft = imuMeasDataLeft(ind3:ind3 + recordLen, :)';

    posFoot = posFoot(ind3:ind3 + recordLen, :)';
    posLeft = posLeft(ind3:ind3 + recordLen, :)';

    posFootStart = posFoot(:, 1);
    posFoot = posFoot - posFootStart;
    posLeft = posLeft - posFootStart;

    [zuptStatusFoot] = zuptDetector(imuMeasFoot, initialGyroBiasFoot);
    [zuptStatusLeft] = zuptDetector(imuMeasLeft, initialGyroBiasLeft);
    imuMeasFootCopy = importIMUMeasFromGithub("../measData/imuMeasRight.csv");
    imuMeasLeftCopy = importIMUMeasFromGithub("../measData/imuMeasLeft.csv");
    positionMeas = importIMUMeasFromGithub("../measData/positionMeas.csv");

    % state vector dimension
    N = 30;
    l = size(imuMeasFoot, 2);

    %--------------------Initialize Covariance Matrix-------------------------
    P = zeros(N, N);
    Q = zeros(24, 24);
    P(1:3, 1:3) = diag(sigma_initial_pos .^ 2);
    P(4:6, 4:6) = diag(sigma_initial_vel .^ 2);
    P(7:9, 7:9) = diag(sigma_initial_att .^ 2);
    P(10:12, 10:12) = diag(sigma_initial_acc_bias .^ 2);
    P(13:15, 13:15) = diag(sigma_initial_gyro_bias .^ 2);
    P(16:30, 16:30) = P(1:15, 1:15);

    Q(1:3, 1:3) = diag(sigma_acc_foot .^ 2);
    Q(4:6, 4:6) = diag(sigma_gyro_foot .^ 2);
    Q(7:9, 7:9) = diag(acc_bias_driving_noise_foot .^ 2);
    Q(10:12, 10:12) = diag(gyro_bias_driving_noise_foot .^ 2);
    Q(13:15, 13:15) = diag(sigma_acc_left .^ 2);
    Q(16:18, 16:18) = diag(sigma_gyro_left .^ 2);
    Q(19:21, 19:21) = diag(acc_bias_driving_noise_left .^ 2);
    Q(22:24, 22:24) = diag(gyro_bias_driving_noise_left .^ 2);

    % measurement matrix
    H1 = zeros(3, N); % zupt @ right foot
    H1(1:3, 4:6) = eye(3);

    H2 = zeros(3, N); % zupt @ left foot
    H2(1:3, 19:21) = eye(3);

    H3 = zeros(6, N); % zupt @ both feet
    H3(1:3, 4:6) = eye(3);
    H3(4:6, 19:21) = eye(3);

    R1 = diag((sigma_vel * [1 1 1]) .^ 2);
    R2 = diag((sigma_vel * [1 1 1 1 1 1]) .^ 2);

    % placeholders to store the estimation results
    stateEstHistoryFoot = zeros(N / 2, l);
    covEstHistoryFoot = zeros(N / 2, l);
    stateEstHistoryLeft = zeros(N / 2, l);
    covEstHistoryLeft = zeros(N / 2, l);

    % initialization of state vector and covariance matrix
    iniAttitudeFoot = [initialRollFoot initialPitchFoot initialYawFoot]';
    Cb2nFoot = Rt2b(iniAttitudeFoot)';
    quatFoot = dcm2q(Cb2nFoot);
    stateEstHistoryFoot(1:3, 1) = [0.018, -0.016, 0.07]'; %ini_pos;
    stateEstHistoryFoot(7:9, 1) = iniAttitudeFoot;
    stateEstHistoryFoot(10:12, 1) = [0.05
                                     -0.02
                                     0.13]';
    stateEstHistoryFoot(13:15, 1) = [initialGyroBiasXFoot
                                     initialGyroBiasYFoot
                                     initialGyroBiasZFoot]';

    iniAttitudeLeft = [initialRollLeft initialPitchLeft initialYawLeft]';
    Cb2nLeft = Rt2b(iniAttitudeLeft)';
    quatLeft = dcm2q(Cb2nLeft);
    stateEstHistoryLeft(1:3, 1) = [0.005, 0.305, 0.08]'; %posLeftInit - posFootInit;
    stateEstHistoryLeft(7:9, 1) = iniAttitudeLeft;
    stateEstHistoryLeft(10:12, 1) = [0.05
                                     -0.02
                                     0.13]';
    stateEstHistoryLeft(13:15, 1) = [initialGyroBiasXLeft
                                     initialGyroBiasYLeft
                                     initialGyroBiasZLeft]';
    % constraint
    L = zeros(2, N);
    L(1:2, 1:2) = -eye(2);
    L(1:2, 16:17) = eye(2);
    gamma = 0.6;
    lastTime = 1;
    tol = 1e-5;

    % run the filter
    for k = 2:l

        % compenstate IMU measurements
        imuFoot = imuMeasFoot(:, k); % + stateEstHistoryFoot(10:15, k-1);
        imuLeft = imuMeasLeft(:, k); % + stateEstHistoryLeft(10:15, k-1);

        % propagate navigation state vectors
        [stateEstHistoryFoot(:, k), quatFoot] = Navigation_equations( ...
            stateEstHistoryFoot(:, k - 1), ...
            imuFoot, ...
            quatFoot, ...
            Ts);
        [stateEstHistoryLeft(:, k), quatLeft] = Navigation_equations( ...
            stateEstHistoryLeft(:, k - 1), ...
            imuLeft, ...
            quatLeft, ...
            Ts);
        % update state transition matrix
        [Fc, Gc] = state_matrix2(quatFoot, imuFoot, quatLeft, imuLeft, Ts);

        % discretize Fc and Gc
        Fd = expm(Fc * Ts);
        Qd = discrete_Q(Fc, Gc, Ts, Q);

        % update the covariance matrix
        P = Fd * P * Fd' + Qd;
        P = (P + P') / 2;

        covEstHistoryFoot(:, k) = sqrt(diag(P(1:15, 1:15)));
        covEstHistoryLeft(:, k) = sqrt(diag(P(16:30, 16:30)));

        %------------------------zero velocity update--------------------------

        if zuptStatusFoot(k) == 1 && zuptStatusLeft(k) == 0

            KZupt = (P * H1') / (H1 * P * H1' + R1);

            zZupt = -stateEstHistoryFoot(4:6, k);

            dxZupt = KZupt * zZupt;

            [stateEstHistoryFoot(:, k), quatFoot] = ...
                comp_internal_states(stateEstHistoryFoot(:, k), dxZupt(1:15), quatFoot);
            [stateEstHistoryLeft(:, k), quatLeft] = ...
                comp_internal_states(stateEstHistoryLeft(:, k), dxZupt(16:30), quatLeft);

            P = (eye(N) - KZupt * H1) * P;
            P = (P + P') / 2;
            covEstHistoryFoot(:, k) = sqrt(diag(P(1:15, 1:15)));
            covEstHistoryLeft(:, k) = sqrt(diag(P(16:30, 16:30)));

        elseif zuptStatusFoot(k) == 0 && zuptStatusLeft(k) == 1
            KZupt = (P * H2') / (H2 * P * H2' + R1);

            zZupt = -stateEstHistoryLeft(4:6, k);

            dxZupt = KZupt * zZupt;

            [stateEstHistoryFoot(:, k), quatFoot] = ...
                comp_internal_states(stateEstHistoryFoot(:, k), dxZupt(1:15), quatFoot);
            [stateEstHistoryLeft(:, k), quatLeft] = ...
                comp_internal_states(stateEstHistoryLeft(:, k), dxZupt(16:30), quatLeft);

            P = (eye(N) - KZupt * H2) * P;
            P = (P + P') / 2;
            covEstHistoryFoot(:, k) = sqrt(diag(P(1:15, 1:15)));
            covEstHistoryLeft(:, k) = sqrt(diag(P(16:30, 16:30)));

        elseif zuptStatusFoot(k) == 1 && zuptStatusLeft(k) == 1
            KZupt = (P * H3') / (H3 * P * H3' + R2);

            zZupt =- [stateEstHistoryFoot(4:6, k);
                      stateEstHistoryLeft(4:6, k)];

            dxZupt = KZupt * zZupt;

            [stateEstHistoryFoot(:, k), quatFoot] = ...
                comp_internal_states(stateEstHistoryFoot(:, k), dxZupt(1:15), quatFoot);
            [stateEstHistoryLeft(:, k), quatLeft] = ...
                comp_internal_states(stateEstHistoryLeft(:, k), dxZupt(16:30), quatLeft);

            P = (eye(N) - KZupt * H3) * P;
            P = (P + P') / 2;
            covEstHistoryFoot(:, k) = sqrt(diag(P(1:15, 1:15)));
            covEstHistoryLeft(:, k) = sqrt(diag(P(16:30, 16:30)));
        end

        % position measurement update
        if mod(k, 30) == 2 && k > 2 && posTrigger %
            sigma_pos = 0.5;
            % yk = [posFoot(:,k);posLeft(:,k)] + mvnrnd(zeros(6,1), sigmaR*eye(6))';
            yk = [positionMeas(round(k / 30), 5:7)'; positionMeas(round(k / 30), 2:4)'];

            Rk = sigma_pos ^ 2 * eye(6);
            Hk = zeros(6, 30);
            Hk(1:3, 1:3) = eye(3);
            Hk(4:6, 16:18) = eye(3);
            zK = yk - [stateEstHistoryFoot(1:3, k);
                       stateEstHistoryLeft(1:3, k)];
            Kk = (P * Hk') / (Hk * P * Hk' + Rk);
            dk = Kk * zK;
            [stateEstHistoryFoot(:, k), quatFoot] = ...
                comp_internal_states(stateEstHistoryFoot(:, k), dk(1:15), quatFoot);
            [stateEstHistoryLeft(:, k), quatLeft] = ...
                comp_internal_states(stateEstHistoryLeft(:, k), dk(16:30), quatLeft);
            P = (eye(N) - Kk * Hk) * P;
            P = (P + P') / 2;
            covEstHistoryFoot(:, k) = sqrt(diag(P(1:15, 1:15)));
            covEstHistoryLeft(:, k) = sqrt(diag(P(16:30, 16:30)));
        end

        %-------------------------constraint update-----------------------------
        Xcon = [stateEstHistoryFoot(:, k); stateEstHistoryLeft(:, k)];
        Pcon = P;
        G = chol(Pcon ^ (-1));
        [U, S, V] = svd(L * G ^ (-1));
        E = V' * G * Xcon;
        Alpha = diag(S' * S);

        if sqrt((L * Xcon)' * (L * Xcon)) > gamma && k - lastTime > 30 && stepTrigger

            lambda = Newton(tol, E, Alpha, gamma);
            % lambda = Bisection(tol,E,Alpha,gamma);

            proj = ((Pcon ^ (-1) + lambda * (L' * L)) ^ (-1)) * (Pcon ^ (-1));
            Xcon = proj * (Xcon);

            Del_lam = Xcon' * (L' * L) * proj / (Xcon' * (L' * L) * ((Pcon ^ (-1) + lambda * (L' * L)) ^ (-1)) * (L' * L) * Xcon);
            JocP = proj - ((Pcon ^ (-1) + lambda * (L' * L)) ^ (-1)) * (L' * L) * Xcon * Del_lam;

            Pcon = JocP * Pcon * JocP';

            stateEstHistoryFoot(1:9, k) = Xcon(1:9);
            stateEstHistoryLeft(1:9, k) = Xcon(16:24);
            P = (Pcon + Pcon') / 2;

            lastTime = k;

        end

    end

    posEstFoot = stateEstHistoryFoot(1:3, :);
    velEstFoot = stateEstHistoryFoot(4:6, :);
    attEstFoot = stateEstHistoryFoot(7:9, :);
    accelBiasEstFoot = stateEstHistoryFoot(10:12, :);
    gyroBiasEstFoot = stateEstHistoryFoot(13:15, :);

    posEstLeft = stateEstHistoryLeft(1:3, :);
    velEstLeft = stateEstHistoryLeft(4:6, :);
    attEstLeft = stateEstHistoryLeft(7:9, :);
    accelBiasEstLeft = stateEstHistoryLeft(10:12, :);
    gyroBiasEstLeft = stateEstHistoryLeft(13:15, :);
    timeStamps = [0:size(imuMeasFoot, 2) - 1] * Ts;

    %%
    % Create positionMeas.csv
    pos_index = [0:length(posEstLeft) - 1]';
    posTable = array2table([pos_index, stateEstHistoryFoot(1:3, :)', stateEstHistoryLeft(1:3, :)'], ...
        'VariableNames', {'time_index', 'pos_left_x', 'pos_left_y', 'pos_left_z', 'pos_right_x', 'pos_right_y', 'pos_right_z'});
    writetable(posTable, fileName); % EKF+ZUPT+posMeas+MaxFoot

    %% plot
    % figure,
    % plot(posFoot(2, :), posFoot(1, :), 'r'); grid on; axis equal; hold on;
    % plot(posLeft(2, :), posLeft(1, :), 'b'); legend('Right Imu', 'Left Imu');
    % title('Ground Track'); xlabel('X(m)'); ylabel('Y(m)');

    % figure,
    % subplot(122),
    % plot(posFoot(2, :), posFoot(1, :), 'r.'); grid on; axis equal; hold on;
    % plot(posEstFoot(2, :), posEstFoot(1, :), 'b.'); legend('Vicon', 'Estimation');
    % title('Right IMU trajectory'); xlabel('X(m)'); ylabel('Y(m)');
    % subplot(121),
    % plot(posLeft(2, :), posLeft(1, :), 'r.'); grid on; axis equal; hold on;
    % plot(posEstLeft(2, :), posEstLeft(1, :), 'b.'); legend('Vicon', 'Estimation');
    % title('Left IMU trajectory'); xlabel('X(m)'); ylabel('Y(m)');

    % figure,
    % subplot(122),
    % plot(posFoot(2, 1:250), posFoot(1, 1:250), 'r'); grid on; axis equal; hold on;
    % plot(posEstFoot(2, 1:250), posEstFoot(1, 1:250), 'b'); legend('Vicon', 'Estimation');
    % title('Right IMU trajectory for first 4 seconds'); xlabel('X(m)'); ylabel('Y(m)');
    % subplot(121),
    % plot(posLeft(2, 1:250), posLeft(1, 1:250), 'r'); grid on; axis equal; hold on;
    % plot(posEstLeft(2, 1:250), posEstLeft(1, 1:250), 'b'); legend('Vicon', 'Estimation');
    % title('Left IMU trajectory for first 4 seconds'); xlabel('X(m)'); ylabel('Y(m)');

    % figure,
    % subplot(211), plot(timeStamps, posEstFoot(1, :), 'b'); hold on; grid on;
    % plot(timeStamps, posFoot(1, :), 'r');
    % legend('Estimation', 'Vicon');
    % title('Right IMU X Position');
    % xlabel('time(sec)'); ylabel('x pos(m)');
    % subplot(212), plot(timeStamps, posEstFoot(2, :), 'b'); hold on; grid on;
    % plot(timeStamps, posFoot(2, :), 'r');
    % legend('Estimation', 'Vicon');
    % title('Right IMU Y Position');
    % xlabel('time(sec)'); ylabel('y pos(m)');

    % figure,
    % subplot(211), plot(timeStamps, posEstLeft(1, :), 'b'); hold on; grid on;
    % plot(timeStamps, posLeft(1, :), 'r');
    % legend('Estimation', 'Vicon');
    % title('Left IMU X Position');
    % xlabel('time(sec)'); ylabel('x pos(m)');
    % subplot(212), plot(timeStamps, posEstLeft(2, :), 'b'); hold on; grid on;
    % plot(timeStamps, posLeft(2, :), 'r');
    % legend('Estimation', 'Vicon');
    % title('Left IMU Y Position');
    % xlabel('time(sec)'); ylabel('y pos(m)');

    % figure,
    % subplot(211), plot(timeStamps, posFoot(1, :) - posEstFoot(1, :), 'b');
    % grid on; hold on; plot(timeStamps, covEstHistoryFoot(1, :), 'r--');
    % plot(timeStamps, -covEstHistoryFoot(1, :), 'r--');
    % title('Right IMU position error');
    % xlabel('time(sec)'); ylabel('x pos error(m)');
    % subplot(212), plot(timeStamps, posFoot(2, :) - posEstFoot(2, :), 'b');
    % grid on; hold on;
    % plot(timeStamps, covEstHistoryFoot(2, :), 'r--');
    % plot(timeStamps, -covEstHistoryFoot(2, :), 'r--');
    % xlabel('time(sec)'); ylabel('y pos error(m)');

    % figure,
    % subplot(211), plot(timeStamps, posLeft(1, :) - posEstLeft(1, :), 'b');
    % grid on; hold on;
    % plot(timeStamps, covEstHistoryLeft(1, :), 'r--');
    % plot(timeStamps, -covEstHistoryLeft(1, :), 'r--');
    % title('Left IMU position error');
    % xlabel('time(sec)'); ylabel('x pos error(m)');
    % subplot(212), plot(timeStamps, posLeft(2, :) - posEstLeft(2, :), 'b');
    % grid on; hold on;
    % plot(timeStamps, covEstHistoryLeft(2, :), 'r--');
    % plot(timeStamps, -covEstHistoryLeft(2, :), 'r--');
    % xlabel('time(sec)'); ylabel('y pos error(m)');

    % figure,
    % plot(timeStamps, vecnorm(posFoot(1:2, :) - posEstFoot(1:2, :)), 'b');
    % grid on; hold on;

    % title('Right IMU horizontal error');
    % xlabel('time(sec)'); ylabel('x pos error(m)');

    % figure,
    % plot(timeStamps, vecnorm(posLeft(1:2, :) - posEstLeft(1:2, :)), 'b');
    % grid on; hold on;

    % title('Left IMU horizontal error');
    % xlabel('time(sec)'); ylabel('x pos error(m)');

end
