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
filename_1 = "./data/LO_FCPE.mat"; % LiDAR Odometry
filename_2 = "./data/VO_FCPE.mat"; % Visual Odometry
filename_1_out = "./results/LO2VO.txt"; % LiDAR Odometry
filename_2_out = "./results/VO_LO.txt"; % Visual Odometry
%% Read LiDAR Odometry and Visual Odometry Data
interval = 3;
data_1 = load(filename_1, '-ascii');
data_2 = load(filename_2, '-ascii');
timestamp_1 = data_1(1 : interval : end, 1);
timestamp_2 = data_2(:, 1);
pose_1 = data_1(1 : interval : end, 2 : 8);
pose_2 = data_2(:, 2 : 8);
%% Coordinate Transformation
R0_1 = quat2rotm(pose_1(1, 4 : 7)); % qw qx qy qz
t0_1 = pose_1(1, 1 : 3);
[m, ~] = size(pose_1);
for i = 1 : m
    pose_1(i, 1 : 3) = R0_1 \ (pose_1(i, 1 : 3)' - t0_1');
    R = R0_1 \ quat2rotm(pose_1(i, 4 : 7)); % qw qx qy qz
    quat = rotm2quat(R); % qw qx qy qz
    pose_1(i, 4 : 7) = quat; % qw qx qy qz
end
R0_2 = quat2rotm(pose_2(1, 4 : 7));
t0_2 = pose_2(1, 1 : 3);
[n, ~] = size(pose_2);
for i = 1 : n
    pose_2(i, 1 : 3) = R0_2 \ (pose_2(i, 1 : 3)' - t0_2');
    R = R0_2 \ quat2rotm(pose_2(i, 4 : 7)); % qw qx qy qz
    quat = rotm2quat(R); % qw qx qy qz
    pose_2(i, 4 : 7) = quat; % qw qx qy qz
end
%% Pose Interpolation
[pose_1_interp, timestamp_1_interp, pose_2_interp, timestamp_2_interp] = poseInterp(pose_1, timestamp_1, pose_2, timestamp_2);
%% Plot to Check Data
figure
hold on
grid on
axis equal
plot3(pose_1_interp(:, 1), pose_1_interp(:, 2), pose_1_interp(:, 3), 'bo-', 'LineWidth', 2)
plot3(pose_2_interp(:, 1), pose_2_interp(:, 2), pose_2_interp(:, 3), 'rs-', 'LineWidth', 2)
xlabel('X / m')
ylabel('Y / m')
zlabel('Z / m')
title('Before Calibration')
legend('LiDAR Odometry', 'Visual Odometry')
view(3)
%% Optimization
fun = @(x)costFunction_L2C_quat_interp(pose_1_interp, pose_2_interp, x);
% options = optimset( 'Display', 'iter', 'MaxFunEvals', 1e6, 'MaxIter', 1e6);
options = optimset('PlotFcns', 'optimplotfval', 'MaxFunEvals', 1e6, 'MaxIter', 1e6);
% Constrained
A = [];
b = [];
Aeq = [];
beq = [];
lb = [-1, -1, -1, -2, -2, -2, -2, 0];
ub = [1, 1, 1, 2, 2, 2, 2, 10];
x0 = (lb + ub) / 2; % x y z (m) qw qx qy qz scale
x0(1, 4 : 7) = [1, 0, 0, 0]; % qw qx qy qz
% x0 = [-0.5, -0.3, -0.02, -pi / 2, -pi / 2, 0, 3]; % Measurement: x y z (m) yaw pitch roll (rad) scale
% Possible Gimbal Lock!
[x,fval,exitflag,output] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, [], options); % Constrained
% x(1, 4 : 7) = normalize(x(1, 4 : 7)); % Do Not Use!!!
x(1, 4 : 7) = x(1, 4 : 7) / sqrt(sum(x(1, 4 : 7).^2));
%% Transform
% R12 = quat2rotm(x(1, 4 : 7));
% t12 = x(1, 1 : 3);
fprintf("LiDAR -> Camera Extrinsic: |\tX\t\t|\tY\t\t|\tZ\t\t|\tqw\t\t|\tqx\t\t|\tqy\t\t|\tqz\t\t|\tScale\t|\n")
fprintf("LiDAR -> Camera Extrinsic: |\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\n", x)
eul = quat2eul(x(1, 4 : 7), 'ZYX');
fprintf("LiDAR -> Camera Extrinsic: |\tX\t\t|\tY\t\t|\tZ\t\t|\tYaw\t\t|\tPitch\t|\tRoll\t|\tScale\t|\n")
fprintf("LiDAR -> Camera Extrinsic: |\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\t%.4f\t|\n", x(1, 1 : 3), eul, x(1, 8))
scale = x(1, 8);
T12 = quat2tform(x(1, 4 : 7));
T12(1 : 3, 4) = x(1, 1 : 3)';
fprintf("T12 = \n")
disp(T12)
fprintf("T12^-1 = \n")
disp(inv(T12))
[m, ~] = size(pose_1_interp);
pose_L2C = zeros(m, 7);
for i = 1 : m
    pose_1_temp = quat2tform(pose_1_interp(i, 4 : 7));
    pose_1_temp(1 : 3, 4) = pose_1_interp(i, 1 : 3)';
    pose_L2C_temp = T12 \ pose_1_temp * T12; % Correct
%     pose_L2C_temp = pose_1_temp * T12; % Wrong !!!
    pose_L2C(i, :) = [pose_L2C_temp(1 : 3, 4)', tform2quat(pose_L2C_temp)];
end
pose_2_interp(:, 1 : 3) = pose_2_interp(:, 1 : 3) * scale;
%% Plot to Check Data
figure
hold on
grid on
axis equal
plot3(pose_1_interp(:, 1), pose_1_interp(:, 2), pose_1_interp(:, 3), 'k^-.', 'LineWidth', 1)
plot3(pose_L2C(:, 1), pose_L2C(:, 2), pose_L2C(:, 3), 'bo-', 'LineWidth', 2)
plot3(pose_2_interp(:, 1), pose_2_interp(:, 2), pose_2_interp(:, 3), 'rs-', 'LineWidth', 2)
xlabel('X / m')
ylabel('Y / m')
zlabel('Z / m')
title('After Calibration')
legend('LiDAR Pose Original', 'LiDAR Pose Transformed', 'Camera Pose with Scale')
view(3)
%% Export Poses for Evaluation
writematrix([timestamp_1_interp, pose_L2C], filename_1_out, 'Delimiter', ' ')
writematrix([timestamp_2_interp, pose_2_interp], filename_2_out, 'Delimiter', ' ')