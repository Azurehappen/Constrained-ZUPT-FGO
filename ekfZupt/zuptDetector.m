function [zuptStatus] = zuptDetector(imuMeas, initialGyroBias)
    %UNTITLED5 Summary of this function goes here
    %   imuMeas: 6 x Num
    %   initialGyroBias: 3 x 1
    gyroMeas = imuMeas(4:6, :) + initialGyroBias;
    accelMeas = imuMeas(1:3, :);
    latitude = 44.986656;
    altitude = -256;
    g = gravity(latitude, altitude);

    gyroNorm = vecnorm(gyroMeas, 2, 1);
    acceNorm = abs(vecnorm(accelMeas, 2, 1)) - g;

    zuptStatus = zeros(size(imuMeas, 2), 1);
    timeStamps = [0:size(imuMeas, 2) - 1] / 60;
    gyroNormAtZupt = [];

    zuptGyroNormThreshold = 0.3;

    zuptAcceNormThreshold = 1.5;

    for i = 1:size(imuMeas, 2)

        if gyroNorm(i) < zuptGyroNormThreshold
            zuptStatus(i) = 1;
            gyroNormAtZupt = [gyroNormAtZupt; timeStamps(i) gyroNorm(i)];
        end

    end

    if 1
        startIndexToLeng = [];
        counter = 0;
        isCounting = false;

        for i = 1:size(zuptStatus, 1)

            if (zuptStatus(i) == 1 && i == 1) || (i > 1 && zuptStatus(i - 1) == 0 && zuptStatus(i) == 1)
                startIndex = i;
                counter = 1;
                isCounting = true;
            elseif isCounting && zuptStatus(i) == 1
                counter = counter + 1;
            elseif zuptStatus(i) == 0 && isCounting && zuptStatus(i - 1) == 1
                startIndexToLeng = [startIndexToLeng; [startIndex counter]];
                counter = 0;
                isCounting = false;
            end

        end

        for i = 1:length(startIndexToLeng)

            if startIndexToLeng(i, 2) < 9
                zuptStatus(startIndexToLeng(i, 1): ...
                    startIndexToLeng(i, 1) + startIndexToLeng(i, 2) - 1) = 0;
            end

        end

    end

    timeStampsUpdated = timeStamps;
    timeStampsUpdated(1, zuptStatus == 0) = nan;
    gyroNormUpdated = gyroNorm;
    gyroNormUpdated(1, zuptStatus == 0) = nan;

end
