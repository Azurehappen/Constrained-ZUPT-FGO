function [zuptStatus] = zuptDetector(imuMeas, initialGyroBias)
    %UNTITLED5 Summary of this function goes here
    %   imuMeas: 6 x Num
    %   initialGyroBias: 3 x 1
    gyroMeas = imuMeas(4:6,:) + initialGyroBias;
    accelMeas = imuMeas(1:3,:);
    latitude = 44.986656;
    altitude = -256;

    g = 9.7803267715*(1+0.001931851353*(sin(latitude))^2) /...
        sqrt(1-0.0066943800229*(sin(latitude))^2);

    % g = gravity(latitude,altitude);

    gyroNorm = vecnorm(gyroMeas, 3, 1);
    acceNorm = abs(abs(vecnorm(accelMeas, 3, 1)) - g);

    zuptStatus = zeros(size(imuMeas,2), 1);
    timeStamps = [0:size(imuMeas,2)-1]/60;
    % gyroNormAtZupt = [];

    zuptGyroNormThreshold = 0.3;  % 0.3(for gait0706); %0.4
                                    % 0.015 for simZUPTAidedINS
    zuptAcceNormThreshold = 0.4;

    for i = 1:size(imuMeas,2)
        if i == 204
            oo = 1;
        end
        if gyroNorm(i) < zuptGyroNormThreshold && acceNorm(i) < zuptAcceNormThreshold
            zuptStatus(i) = 1;
            % gyroNormAtZupt = [gyroNormAtZupt;timeStamps(i) gyroNorm(i)];
        end

    end

    % % Step 1: Remove short interruptions (single or double `0`s between `1`s)
    % min_zero_gap = 4;  % Allow up to 2 consecutive zero misdetections
    % 
    % zero_count = 0;
    % for i = 2:size(zuptStatus,1)-1
    %     if zuptStatus(i) == 0
    %         zero_count = zero_count + 1;
    %     else
    %         if zero_count > 0 && zero_count <= min_zero_gap
    %             % Fix misdetection (convert small zero gaps back to `1`s)
    %             zuptStatus(i-zero_count:i-1) = 1;
    %         end
    %         zero_count = 0; % Reset counter
    %     end
    % end
    % 
    % % Step 2: Track sequences again
    % startIndexToLeng = [];
    % counter = 0;
    % isCounting = false;
    % 
    % for i = 1:size(zuptStatus,1)
    %     if (zuptStatus(i) == 1 && i == 1) || (i > 1 && zuptStatus(i-1) == 0 && zuptStatus(i) == 1)
    %         startIndex = i;
    %         counter = 1;
    %         isCounting = true;
    %     elseif isCounting && zuptStatus(i) == 1
    %         counter = counter + 1;
    %     elseif zuptStatus(i) == 0 && isCounting && zuptStatus(i-1) == 1 
    %         startIndexToLeng = [startIndexToLeng; [startIndex, counter]];
    %         counter = 0;
    %         isCounting = false;
    %     end
    % end
    % 
    % % Step 3: Remove sequences shorter than 9 samples
    % for i = 1:length(startIndexToLeng)
    %     if startIndexToLeng(i,2) < 9
    %         zuptStatus(startIndexToLeng(i,1):startIndexToLeng(i,1) + startIndexToLeng(i,2)-1) = 0;
    %     end
    % end

end

