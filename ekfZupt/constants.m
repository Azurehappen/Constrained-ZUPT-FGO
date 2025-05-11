%-------------------------------------------------------------------------
%  Noise Statistics
%-------------------------------------------------------------------------
g = 9.8172;

%acc_bias_driving_noise = 0.03*1e-3*g*ones(3,1);
%gyro_bias_driving_noise = 10/3600*pi/180*ones(3,1);

acc_bias_driving_noise_foot = [0.0014; 0.00065; 0.00015];
gyro_bias_driving_noise_foot = [0.00117; 0.000637; 0.000659] / 180 * pi;
acc_bias_driving_noise_left = [0.005; 0.005; 0.005];
gyro_bias_driving_noise_left = [0.005; 0.05; 0.05] / 180 * pi;

%acc_bias_driving_noise = 0.03*ones(3,1);
%gyro_bias_driving_noise = 0.0038*pi/180*ones(3,1);

sigma_initial_pos = 0.01 * ones(3, 1);
sigma_initial_vel = 0.05 * ones(3, 1);
sigma_initial_att = (pi / 180 * [10 10 10]');
sigma_initial_acc_bias = 0.01 * ones(3, 1);
sigma_initial_gyro_bias = 0.002 * ones(3, 1);

% sigma_acc = 120*1e-6*g*[1 1 1]';
% sigma_gyro = 0.007/180*pi*[1 1 1]'*pi/180;
%sigma_acc_foot = 120*1e-6*g*[1 1 1]';% 0.3*[1 1 1]';
%sigma_gyro_foot = deg2rad(0.007)*[1 1 1]'; % 0.5/180*pi*[1 1 1]'*pi/180
sigma_acc_foot = [0.004; 0.00113; 0.0009]; % 50*0.002*[1 1 1]';    0.3*[1 1 1]';
sigma_gyro_foot = [0.00563; 0.00456; 0.00448] / 180 * pi; % 200*deg2rad(20)/3600*[1 1 1]';    0.5*[1 1 1]'*pi/180;

%sigma_acc_shin = 120*1e-6*g*[1 1 1]';% 0.3*[1 1 1]';
%sigma_gyro_shin = deg2rad(0.007)*[1 1 1]'; % 0.5/180*pi*[1 1 1]'*pi/180
sigma_acc_left = [0.00012; 0.00012; 0.00012]; % 0.3*[1 1 1]';
sigma_gyro_left = [0.007; 0.007; 0.007] / 180 * pi; % 0.5*[1 1 1]'*pi/180;
% sigma_acc_shin = 120*1e-6*g*[1 1 1]';
% sigma_gyro_shin = 0.007/180*pi*[1 1 1]'*pi/180;
% 0.05/0.1

sigma_pos = 0.50; % measurement noise
sigma_vel = 0.012;

tau_g = 300;
tau_a = 100;

%-------------------------------------------------------------------------
%  Initialization
%-------------------------------------------------------------------------
ini_roll = 0;
ini_pitch = 0;
ini_yaw = 0;
ini_pos = [0 0 0]'; % 3x1

%-------------------------------------------------------------------------
%  constants
%-------------------------------------------------------------------------
Ts = 1/60; % sampling time interval
latitude = 44.986656;
altitude = -256;
g = gravity(latitude, altitude);
