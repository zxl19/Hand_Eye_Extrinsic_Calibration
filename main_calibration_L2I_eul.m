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
% INS Coordinate
% x (right)
% y (front)
% z (up)
%% Pose Filename Setup
filename_1 = "pose1_3.csv"; % LiDAR Odometry
filename_2 = "pose2_3.csv"; % INS
%% Read LiDAR Odometry and INS Data
[timestamp_1, pose_1] = readLO(filename_1);
[timestamp_2, pose_2] = readNovatel(filename_2);
%% Data Synchronization or Pose Interpolation (TODO)
threshold = 0.005;
flag = true; % Print Synchronized Timestamp
[pose_1_sync, timestamp_1_sync, pose_2_sync, timestamp_2_sync] = sync(pose_1, timestamp_1, pose_2, timestamp_2, threshold, flag);
%% Coordinate Transformation
R0_1 = quat2rotm(pose_1_sync(1, 4 : 7)); % qw qx qy qz
t0_1 = pose_1_sync(1, 1 : 3);
[m, ~] = size(pose_1_sync);
for i = 1 : m
    pose_1_sync(i, 1 : 3) = R0_1 \ (pose_1_sync(i, 1 : 3)' - t0_1');
    R = R0_1 \ quat2rotm(pose_1_sync(i, 4 : 7)); % qw qx qy qz
    eul = rotm2eul(R, 'ZYX'); % rad
    pose_1_sync(i, 4 : 6) = eul; % ZYX
end
pose_1_sync(:, 7) = [];
[X, Y, ~] = deg2utm(pose_2_sync(:, 1), pose_2_sync(:, 2));
pose_2_sync(:, 1) = X;
pose_2_sync(:, 2) = Y;
R0_2 = eul2rotm(pose_2_sync(1, 4 : 6), 'ZYX'); % ZYX
t0_2 = pose_2_sync(1, 1 : 3);
[n, ~] = size(pose_2_sync);
for i = 1 : n
    pose_2_sync(i, 1 : 3) = R0_2 \ (pose_2_sync(i, 1 : 3)' - t0_2');
    R = R0_2 \ eul2rotm(pose_2_sync(i, 4 : 6), 'ZYX'); % ZYX
    eul = rotm2eul(R, 'ZYX'); % rad
    pose_2_sync(i, 4 : 6) = eul; % ZYX
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
legend('LiDAR Odometry', 'INS')
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
legend('LiDAR      Roll', 'LiDAR      Pitch', 'INS Roll', 'INS Pitch', 'LiDAR      Yaw', 'INS Yaw', 'Location', 'SouthWest')
%% Optimization
fun = @(x)costFunction_L2I_eul(pose_1_sync, pose_2_sync, x);
options = optimset( 'Display', 'iter', 'MaxFunEvals', 1e6, 'MaxIter', 1e6);
% Constrained
A = [];
b = [];
Aeq = [];
beq = [];
lb = [-1, -1, -1, -pi, -pi, -pi];
ub = [1, 1, 1, pi, pi, pi];
x0 = (lb + ub)/2; % x y z (m) yaw pitch roll (rad)
% x0 = [0.35, 0, -0.13, pi / 2, 0, 0]; % Measurement: x y z (m) yaw pitch roll (rad)
[x,fval,exitflag,output] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, [], options); % Constrained
% [x,fval,exitflag,output] = fminsearch(fun, x0, options); % Search
% [x,fval,exitflag,output] = fminunc(fun, x0, options); % Unconstrained
%% Transform
% R12 = eul2rotm(x(1, 4 : 6), 'ZYX');
% t12 = x(1, 1 : 3)';
fprintf("LiDAR -> INS Extrinsic: |\tX\t\t|\tY\t\t|\tZ\t\t|\tYaw\t\t|\tPitch\t|\tRoll\t|\n")
fprintf("LiDAR -> INS Extrinsic: |\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\n", x)
T12 = eul2tform(x(1, 4 : 6), 'ZYX');
T12(1 : 3, 4) = x(1, 1 : 3)';
fprintf("T12 = \n")
disp(T12)
fprintf("T12^-1 = \n")
disp(inv(T12))
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
legend('LiDAR Pose Original', 'LiDAR Pose Transformed', 'INS', 'Location', 'NorthEast')
xlabel('X / m')
ylabel('Y / m')
zlabel('Z / m')