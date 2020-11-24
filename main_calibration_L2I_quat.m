close all
clear
clc
format long
%% Abstract
% LiDAR Coordinate in A-LOAM:
% x (back)
% y (right)
% z (up)
% Camera Coordinate
% x (right)
% y (down)
% z (front)
% GPS/IMU Coordinate
% x (right)
% y (front)
% z (up)
%% Pose Filename Setup
filename_1 = "pose1.csv"; % LiDAR Odometry
filename_2 = "pose2.csv"; % GPS/IMU
%% Read LiDAR Odometry and GPS/IMU Data
T_1 = readtable(filename_1);
T_2 = readtable(filename_2);
timestamp_1 = T_1{:, 1} * 1e-9; % s
timestamp_2 = T_2{:, 1} * 1e-9; % s
pose_1 = T_1{:, 7 : 13}; % x y z qx qy qz qw
latitude = T_2{:, 31};
longitude = T_2{:, 33};
altitude = T_2{:, 35};
roll = deg2rad(T_2{:, 45}); % rad
pitch = deg2rad(T_2{:, 47}); % rad
azimuth = deg2rad(T_2{:, 49}); % rad
pose_2 = [latitude, longitude, altitude, -azimuth, pitch, roll]; % latitude longitude altitude yaw pitch roll
%% Data Synchronization or Pose Interpolation (TODO)
threshold = 0.005;
flag = true; % Print Synchronized Timestamp
[pose_1_sync, timestamp_1_sync, pose_2_sync, timestamp_2_sync] = sync(pose_1, timestamp_1, pose_2, timestamp_2, threshold, flag);
%% Coordinate Transformation
R0_1 = quat2rotm(pose_1_sync(1, [7, 4 : 6])); % qw qx qy qz
t0_1 = pose_1_sync(1, 1 : 3);
[m, ~] = size(pose_1_sync);
for i = 1 : m
    pose_1_sync(i, 1 : 3) = R0_1 \ (pose_1_sync(i, 1 : 3)' - t0_1');
    R = R0_1 \ quat2rotm(pose_1_sync(i, [7, 4 : 6])); % qw qx qy qz
    quat = rotm2quat(R); % qw qx qy qz
    pose_1_sync(i, 4 : 7) = quat; % qw qx qy qz
end
[X, Y, ~] = deg2utm(pose_2_sync(:, 1), pose_2_sync(:, 2));
pose_2_sync(:, 1) = X;
pose_2_sync(:, 2) = Y;
R0_2 = eul2rotm(pose_2_sync(1, 4 : 6), 'ZYX'); % ZYX
t0_2 = pose_2_sync(1, 1 : 3);
[n, ~] = size(pose_2_sync);
for i = 1 : n
    pose_2_sync(i, 1 : 3) = R0_2 \ (pose_2_sync(i, 1 : 3)' - t0_2');
    R = R0_2 \ eul2rotm(pose_2_sync(i, 4 : 6), 'ZYX');
    quat = rotm2quat(R); % qw qx qy qz
    pose_2_sync(i, 4 : 7) = quat; % qw qx qy qz
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
%% Optimization
fun = @(x)costFunction_L2I_quat(pose_1_sync, pose_2_sync, x);
% Constrained
A = [];
b = [];
Aeq = [];
beq = [];
lb = [-1, -1, -1, -2, -2, -2, -2]; % x y z qw qx qy qz
ub = [1, 1, 1, 2, 2, 2, -2];
x0 = (lb + ub)/2;
x0(1, 4 : 7) = [1, 0, 0, 0];% qw qx qy qz
% x0 = [0.35, 0, -0.13, pi / 2, 0, 0]; % Measurement: x y z (m) yaw pitch roll (rad)
[x, fval, exitflag, output] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub); % Constrained
% x(1, 4 : 7) = normalize(x(1, 4 : 7)); % Do Not Use!!!
x(1, 4 : 7) = x(1, 4 : 7) / sqrt(sum(x(1, 4 : 7).^2));
fprintf("LiDAR -> GPS/IMU Extrinsic: |\tX\t\t|\tY\t\t|\tZ\t\t|\tqw\t\t|\tqx\t\t|\tqy\t\t|\tqz\t\t|\n")
fprintf("LiDAR -> GPS/IMU Extrinsic: |\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\n", x)
%% Transform
R12 = quat2rotm(x(1, 4 : 7));
t12 = x(1, 1 : 3);
T12 = quat2tform(x(1, 4 : 7));
T12(1 : 3, 4) = x(1, 1 : 3)';
fprintf("T12 = \n")
disp(T12)
fprintf("T12^-1 = \n")
disp(inv(T12))
[m, ~] = size(pose_1_sync);
pose_L2I = zeros(m, 7);
for i = 1 : m
    pose_1_temp = quat2tform(pose_1_sync(i, 4 : 7));
    pose_1_temp(1 : 3, 4) = pose_1_sync(i, 1 : 3)';
    pose_L2I_temp = T12 \ pose_1_temp * T12; % Correct
%     pose_L2I_temp = pose_1_temp * T12; % Wrong !!!
    pose_L2I(i, :) = [pose_L2I_temp(1 : 3, 4)', tform2quat(pose_L2I_temp)];
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
xlabel('X / m')
ylabel('Y / m')
zlabel('Z / m')