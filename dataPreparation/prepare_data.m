% Prepare data
% IMU right and left, 
load("ImuFootLeft.mat");

% These are ground truth of foot.
load("posFoot.mat");
load("posLeft.mat");

recordLen = 2*60*60;
static_begin = 1536;static_end = 5020;
moving_begin = 5028;
moving_end = moving_begin + recordLen;

% 1:6 -> acc, gyro
imuMeasDataRight = ImuFootLeft(:,1:6);

% 7:12 -> acc, gyro
imuMeasDataLeft = ImuFootLeft(:,7:12);

imuMeasDataRight_static = imuMeasDataRight(static_begin:static_end,:);
imuMeasDataLeft_static = imuMeasDataLeft(static_begin:static_end,:);

noise_right = imuMeasDataRight_static' - mean(imuMeasDataRight_static,1)';
noise_left = imuMeasDataLeft_static' - mean(imuMeasDataLeft_static,1)';

std_right = std(noise_right,0,2);
std_left = std(noise_left,0,2);
% Time axis (assuming sample index as x-axis)
t = 1:size(noise_right, 2);
% Labels
labels = {'Acc X', 'Acc Y', 'Acc Z', 'Gyro X', 'Gyro Y', 'Gyro Z'};
% Right IMU noise figure
figure(1);
for i = 1:6
    subplot(3, 2, i);
    hold on;
    plot(t, noise_right(i, :), 'b'); % Plot noise
    plot(t, std_right(i) * ones(size(t)), 'r--', 'LineWidth', 1.2); % +std
    plot(t, -std_right(i) * ones(size(t)), 'r--', 'LineWidth', 1.2); % -std
    title(['Right IMU - ', labels{i}]);
    xlabel('Sample Index');
    ylabel('Noise');
    grid on;
    hold off;
end

% Left IMU noise figure
figure(2);
for i = 1:6
    subplot(3, 2, i);
    hold on;
    plot(t, noise_left(i, :), 'b'); % Plot noise
    plot(t, std_left(i) * ones(size(t)), 'r--', 'LineWidth', 1.2); % +std
    plot(t, -std_left(i) * ones(size(t)), 'r--', 'LineWidth', 1.2); % -std
    title(['Left IMU - ', labels{i}]);
    xlabel('Sample Index');
    ylabel('Noise');
    grid on;
    hold off;
end

imuMeasDataRight_move = imuMeasDataRight(moving_begin:moving_end,:);
imuMeasDataLeft_move = imuMeasDataLeft(moving_begin:moving_end,:);

posRight_static = posFoot(static_begin:static_end,:);
posLeft_static = posLeft(static_begin:static_end,:);

posRight_gt_move = posFoot(moving_begin:moving_end,:);
posLeft_gt_move = posLeft(moving_begin:moving_end,:);

% create time array
time_index = 0 : 1 : size(imuMeasDataRight_move,1)-1;

noise_std = 0.3;  % Standard deviation of Gaussian noise in meters
bias_values = [-1.5, -1, -0.6, -0.3, 0, 0.3, 0.6, 1, 1.5];  % Possible bias values

posMeasRight = []; 
posMeasLeft = [];
pos_index = [];

% Step 1: Add Gaussian noise to selected measurement epochs
for i = 2:length(time_index)
    if mod(time_index(i), 30) ~= 0  % Ensure measurements happen at fixed intervals
        continue;
    end
    
    % Add Gaussian noise
    noisyRight = posRight_gt_move(i,:) + noise_std * randn(1, size(posRight_gt_move, 2));
    noisyLeft = posLeft_gt_move(i,:) + noise_std * randn(1, size(posLeft_gt_move, 2));

    % Store the measurement index
    pos_index = [pos_index; i];

    % Append to measurement lists
    posMeasRight = [posMeasRight; noisyRight];
    posMeasLeft = [posMeasLeft; noisyLeft];
end

% Step 2: Introduce Outliers in 30% of Measurements
num_measurements = length(pos_index);
num_outliers = round(num_measurements * 0.55);  % 55% outliers
outlier_indices = randperm(num_measurements, num_outliers);  % Randomly select indices

for j = outlier_indices
    bias_right = bias_values(randi(length(bias_values))) * (1 + rand());  % Random bias scaling
    bias_left = bias_values(randi(length(bias_values))) * (1 + rand());

    posMeasRight(j, :) = posMeasRight(j, :) + bias_right;  % Apply outlier
    posMeasLeft(j, :) = posMeasLeft(j, :) + bias_left;
end

% take the average of stationary imu data to calculate initial gyro bias
initialGyroBiasXRight = mean(imuMeasDataRight_static(:,4));
initialGyroBiasYRight = mean(imuMeasDataRight_static(:,5));
initialGyroBiasZRight = mean(imuMeasDataRight_static(:,6));
initialGyroBiasRight = [initialGyroBiasXRight;
                       initialGyroBiasYRight;
                       initialGyroBiasZRight];

initialGyroBiasXLeft = mean(imuMeasDataLeft_static(:,4));
initialGyroBiasYLeft = mean(imuMeasDataLeft_static(:,5));
initialGyroBiasZLeft = mean(imuMeasDataLeft_static(:,6));
initialGyroBiasLeft = [initialGyroBiasXLeft;
                       initialGyroBiasYLeft;
                       initialGyroBiasZLeft];

initialAccBiasXLeft = mean(imuMeasDataLeft_static(:,1));
initialAccBiasYLeft = mean(imuMeasDataLeft_static(:,2));
initialAccBiasZLeft = mean(imuMeasDataLeft_static(:,3));
initialAccBiasLeft = [initialAccBiasXLeft;
                       initialAccBiasYLeft;
                       initialAccBiasZLeft];

initialAccBiasXRight = mean(imuMeasDataRight_static(:,1));
initialAccBiasYRight = mean(imuMeasDataRight_static(:,2));
initialAccBiasZRight = mean(imuMeasDataRight_static(:,3));
initialAccBiasRight = [initialAccBiasXRight;
                       initialAccBiasYRight;
                       initialAccBiasZRight];

% Test static:
gyroMeas = imuMeasDataRight_move(:,4:6)' - initialGyroBiasRight;
accelMeas = imuMeasDataRight_move(:,1:3)' - initialAccBiasRight;
gyroNorm = vecnorm(gyroMeas, 3, 1);
acceNorm = vecnorm(accelMeas, 3, 1);

[zeroVelStatusRight] = zuptDetector(imuMeasDataRight_move', ...
    initialGyroBiasRight);
[zeroVelStatusLeft] = zuptDetector(imuMeasDataLeft_move', ...
    initialGyroBiasLeft);

% make csv files.
% Create imuMeasRight.csv
imuRightTable = array2table([time_index', imuMeasDataRight_move, zeroVelStatusRight], ...
    'VariableNames', {'time_index', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'is_zero_vel'});
writetable(imuRightTable, "../measData/imuMeasRight.csv");

% Create imuMeasLeft.csv
imuLeftTable = array2table([time_index', imuMeasDataLeft_move, zeroVelStatusLeft], ...
    'VariableNames', {'time_index', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'is_zero_vel'});
writetable(imuLeftTable, "../measData/imuMeasLeft.csv");

% Create positionMeas.csv
posTable = array2table([pos_index, posMeasLeft, posMeasRight], ...
    'VariableNames', {'time_index', 'pos_left_x', 'pos_left_y', 'pos_left_z', 'pos_right_x', 'pos_right_y', 'pos_right_z'});
writetable(posTable, "../measData/positionMeas.csv");

posGtTable = array2table([time_index', posLeft_gt_move, posRight_gt_move], ...
    'VariableNames', {'time_index', 'pos_left_x', 'pos_left_y', 'pos_left_z', 'pos_right_x', 'pos_right_y', 'pos_right_z'});
writetable(posGtTable, "../measData/positionGt.csv");