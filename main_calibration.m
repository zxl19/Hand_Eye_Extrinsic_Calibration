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
filename_3 = "pose4.txt"; % Camera
%% Read LiDAR Odometry and GPS/IMU Data
T_1 = readtable(filename_1);
T_2 = readtable(filename_2);
T_3 = readtable(filename_3);
timestamp_1 = T_1{:, 1} * 10^-9; % s
timestamp_2 = T_2{:, 1} * 10^-9; % s
timestamp_3 = T_3{:, 1}; % s
pose_1 = T_1{:, 7 : 13}; % x y z qx qy qz qw
latitude = T_2{:, 31};
longitude = T_2{:, 33};
altitude = T_2{:, 35};
roll = deg2rad(T_2{:, 45}); % rad
pitch = deg2rad(T_2{:, 47}); % rad
azimuth = deg2rad(T_2{:, 49}); % rad
pose_2 = [latitude, longitude, altitude, azimuth, pitch, roll]; % latitude longitude altitude yaw pitch roll
pose_3 = [T_3{:, 6 : 8}, T_3{:, 2 : 5}]; % x y z qw qx qy qz
%% Data Synchronization or Pose Interpolation (TODO)
threshold = 0.002;
flag = true; % Print Synchronized Timestamp
% [pose_1_sync, timestamp_1_sync, pose_2_sync, timestamp_2_sync] = sync(pose_1, timestamp_1, pose_2, timestamp_2, threshold, flag);
[pose_2_sync, timestamp_2_sync, pose_3_sync, timestamp_3_sync] = sync(pose_2, timestamp_2, pose_3, timestamp_3, threshold, flag);
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
pose_3_sync(:, 1 : 3) = pose_3_sync(:, 1 : 3) - pose_3_sync(1, 1 : 3);
[p, ~] = size(pose_3_sync);
R0 = quat2rotm(pose_3_sync(1, 4 : 7)); % qw qx qy qz
for i = 1 : p
    R = R0 \ quat2rotm(pose_1_sync(i, 4 : 7)); % qw qx qy qz
    eul = rotm2eul(R, 'ZYX'); % rad
    pose_3_sync(i, 4 : 6) = eul; % XYZ
end
pose_3_sync(:, 7) = [];
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
mode = ''; % Constrained/Unconstrained/Search
fun = @(x)costFunction(pose_1_sync, pose_2_sync, x);
% x0 = [0, 0, -0.15, 0, 0, -pi/2]; % Initial Value: x y z (m) roll pitch yaw (rad)
% options = optimset('Display', 'iter', 'FunValCheck', 'on', 'MaxFunEvals', 1e6, 'TolFun', 1e-6, 'TolX', 1e-6);
options = optimset('Display', 'iter', 'FunValCheck', 'on');
% [x,fval,exitflag,output] = fminsearch(fun, x0, options); % Search
% [x,fval,exitflag,output] = fminunc(fun, x0, options); % Unconstrained
% Constrained
A = [];
b = [];
Aeq = [];
beq = [];
lb = [-1, -1, -1, -pi, -pi, -pi];
ub = [1, 1, 1, pi, pi, pi];
x0 = (lb + ub)/2;
[x,fval,exitflag,output] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub); % Constrained
fprintf("LiDAR -> GPS/IMU Extrinsic: |\tX\t\t|\tY\t\t|\tZ\t\t|\tYaw\t\t|\tPitch\t|\tRoll\t|\n")
fprintf("LiDAR -> GPS/IMU Extrinsic: |\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\n", x)
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
    pose_L2I_temp = T12 \ pose_1_temp * T12; % Correct
%     pose_L2I_temp = pose_1_temp * T12; % Wrong !!!
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
xlabel('X / m')
ylabel('Y / m')
zlabel('Z / m')