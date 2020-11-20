close all
clear
clc
format long
%% Parameter Setup
filename_1 = "pose1.csv"; % LiDAR Odometry
filename_2 = "pose2.csv"; % GPS/IMU
%% Read LiDAR Odometry and GPS/IMU Data
T_1 = readtable(filename_1);
T_2 = readtable(filename_2);
timestamp_1 = T_1{:, 1} * 10^-9; % s
timestamp_2 = T_2{:, 1} * 10^-9; % s
pose_1 = T_1{:, 7 : 13}; % x y z qx qy qz qw
latitude = T_2{:, 31};
longitude = T_2{:, 33};
altitude = T_2{:, 35};
roll = deg2rad(T_2{:, 45}); % rad
pitch = deg2rad(T_2{:, 47}); % rad
azimuth = deg2rad(T_2{:, 49}); % rad
pose_2 = [latitude, longitude, altitude, roll, pitch, azimuth]; % latitude longitude altitude row pitch azimuth
%% Synchronize Data or Pose Interpolation (TODO)
threshold = 0.005;
[pose_1_sync, timestamp_1_sync, pose_2_sync, timestamp_2_sync] = sync(pose_1, timestamp_1, pose_2, timestamp_2, threshold);
%% Coordinate Transformation
pose_1_sync(:, 1 : 3) = pose_1_sync(:, 1 : 3) - pose_1_sync(1, 1 : 3);
[m, ~] = size(pose_1_sync);
R0 = quat2rotm(pose_1_sync(1, [end, end - 3 : end - 1])); % qw qx qy qz
for i = 1 : m
    R = R0 \ quat2rotm(pose_1_sync(i, [end, end - 3 : end - 1])); % qw qx qy qz
    eul = rotm2eul(R, 'ZYX'); % rad
    pose_1_sync(i, 4 : 6) = eul; % XYZ
end
pose_1_sync(:, 7) = [];
[X, Y, ~] = deg2utm(pose_2_sync(:, 1), pose_2_sync(:, 2));
pose_2_sync(:, 1) = X;
pose_2_sync(:, 2) = Y;
pose_2_sync(:, 1 : 3) = pose_2_sync(:, 1 : 3) - pose_2_sync(1, 1 : 3);
[n, ~] = size(pose_2_sync);
R0 = eul2rotm(pose_2_sync(1, end - 2 : end), 'ZYX'); % XYZ
for i = 1 : n
    R = R0 \ eul2rotm(pose_2_sync(i, end - 2 : end), 'ZYX');
    eul = rotm2eul(R, 'ZYX');
    pose_2_sync(i, end - 2 : end) = eul; % XYZ
end
%% Plot to Check Data
figure
hold on
grid on
axis equal
plot3(pose_1_sync(:, 1), pose_1_sync(:, 2), pose_1_sync(:, 3), 'bo-', 'LineWidth', 2)
plot3(pose_2_sync(:, 1), pose_2_sync(:, 2), pose_2_sync(:, 3), 'rs-', 'LineWidth', 2)
xlabel('X / m')
ylabel('Y / m')
zlabel('Z / m')
title('Trajectories')
legend('LiDAR Odometry', 'GPS/IMU')
figure
hold on
grid on
colororder({'b','r'})
yyaxis left
plot(pose_1_sync(:, 4), '-s', 'LineWidth', 2)
plot(pose_1_sync(:, 5), '-o', 'LineWidth', 2)
yyaxis right
plot(pose_1_sync(:, 6), '-^', 'LineWidth', 2)
yyaxis left
plot(pose_2_sync(:, 4), '--s', 'LineWidth', 2)
plot(pose_2_sync(:, 5), '--o', 'LineWidth', 2)
ylabel('Euler Angle / rad')
yyaxis right
plot(pose_2_sync(:, 6), '--^', 'LineWidth', 2)
xlabel('Index')
ylabel('Euler Angle / rad')
title('Trajectories')
legend('LiDAR      Roll', 'LiDAR      Pitch', 'GPS/IMU Roll', 'GPS/IMU Pitch', 'LiDAR      Yaw', 'GPS/IMU Yaw', 'Location', 'SouthWest')
%% Optimization
fun = @(x)costFunction(pose_1_sync, pose_2_sync, x);
x0 = [0, 0, 0, 0, 0, -pi/2]; % Initial Value
% options = optimset('Display', 'iter', 'FunValCheck', 'on', 'MaxFunEvals', 1e6, 'TolFun', 1e-6, 'TolX', 1e-6);
options = optimset('Display', 'iter', 'FunValCheck', 'on');
% [x,fval,exitflag,output] = fminsearch(fun, x0, options);
[x,fval,exitflag,output] = fminunc(fun, x0, options);
fprintf("LiDAR -> GPS/IMU Extrinsic: %f\t%f\t%f\t%f\t%f\t%f\n", x)
%% Transform
R12 = eul2rotm(x(1, 4 : 6), 'ZYX');
t12 = x(1, 1 : 3);
T12 = eul2tform(x(1, 4 : 6), 'ZYX');
T12(1 : 3, 4) = x(1, 1 : 3)';
[m, ~] = size(pose_1_sync);
pose_L2I = zeros(m, 6);
for i = 1 : m
    pose_1_temp = eul2tform(pose_1_sync(i, 4 : 6), 'ZYX');
    pose_1_temp(1 : 3, 4) = pose_1_sync(i, 1 : 3)';
    pose_L2I_temp = T12 \ pose_1_temp * T12;
%     pose_L2I_temp = pose_1_temp * T12; % Not Working
    pose_L2I(i, :) = [pose_L2I_temp(1 : 3, 4)', tform2eul(pose_L2I_temp, 'ZYX')];
end
%% Plot to Check Data
figure
hold on
grid on
axis equal
plot3(pose_1_sync(:, 1), pose_1_sync(:, 2), pose_1_sync(:, 3), 'k^-.', 'LineWidth', 1)
plot3(pose_L2I(:, 1), pose_L2I(:, 2), pose_L2I(:, 3), 'bo-', 'LineWidth', 2)
plot3(pose_2_sync(:, 1), pose_2_sync(:, 2), pose_2_sync(:, 3), 'rs-', 'LineWidth', 2)
legend('LiDAR Pose Original', 'LiDAR Pose Transformed', 'GPS/IMU', 'Location', 'NorthEast')