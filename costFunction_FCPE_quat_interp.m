function loss = costFunction_FCPE_quat_interp(pose_1_13, pose_3_13, pose_2_23, pose_3_23, pose_1_12, pose_2_12, x)
loss = 0;
x_13 = x(1, 1 : 7); % LiDAR to INS
x_23 = x(1, 8 : 15); % Camera to INS
x_12 = x(1, 16 : 23); % LiDAR to Camera
mu = [1, 1, 1, 1, 1]; % weight
%% LiDAR to INS
quat_13 = x_13(1, 4 : 7);
quat_13 = quat_13 / sqrt(sum(quat_13.^2));
R_13 = quat2rotm(quat_13);
t_13 = x_13(1, 1 : 3)';
[m, ~] = size(pose_1_13);
for i = 2 : m
    [R_1, t_1] = calcRelativePose_quat(pose_1_13(i - 1, :), pose_1_13(i, :));
    [R_3, t_3] = calcRelativePose_quat(pose_3_13(i - 1, :), pose_3_13(i, :));
    loss = loss + mu(1) * norm(R_1 * R_13 - R_13 * R_3, 2) + ...
        mu(2) * norm(R_1 * t_13 + t_1 - R_13 * t_3 - t_13, 2);
end
%% Camera to INS
quat_23 = x_23(1, 4 : 7);
quat_23 = quat_23 / sqrt(sum(quat_23.^2));
R_23 = quat2rotm(quat_23);
t_23 = x_23(1, 1 : 3)';
s_23 = x_23(1, 8);
[n, ~] = size(pose_2_23);
for j = 2 : n
    [R_2, t_2] = calcRelativePose_quat(pose_2_23(j - 1, :), pose_2_23(j, :));
    [R_3, t_3] = calcRelativePose_quat(pose_3_23(j - 1, :), pose_3_23(j, :));
    loss = loss + mu(1) * (norm(R_2 * R_23 - R_23 * R_3, 2))^2 + ...
        mu(2) * (norm(R_2 * t_23 + t_2 * s_23 - R_23 * t_3 - t_23, 2)^2);
%     loss = loss + mu(1) * norm(R_2 * R_23 - R_23 * R_3, 2) + ...
%         mu(2) * norm(R_2 * t_23 + t_2 * s_23 - R_23 * t_3 - t_23, 2); % Worse
end
%% LiDAR to Camera
quat_12 = x_12(1, 4 : 7);
quat_12 = quat_12 / sqrt(sum(quat_12.^2));
R_12 = quat2rotm(quat_12);
t_12 = x_12(1, 1 : 3)';
s_12 = x_12(1, 8);
[p, ~] = size(pose_1_12);
for k = 2 : p
    [R_1, t_1] = calcRelativePose_quat(pose_1_12(k - 1, :), pose_1_12(k, :));
    [R_2, t_2] = calcRelativePose_quat(pose_2_12(k - 1, :), pose_2_12(k, :));
    loss = loss + mu(1) * (norm(R_1 * R_12 - R_12 * R_2, 2)^2) + ...
        mu(2) * (norm(R_1 * t_12 + t_1 - R_12 * t_2 * s_12 - t_12, 2)^2);
%     loss = loss + mu(1) * norm(R_1 * R_12 - R_12 * R_2, 2) + ...
%         mu(2) * norm(R_1 * t_12 + t_1 - R_12 * t_2 * s_12 - t_12, 2); % Worse
end
%% Between Extrinsics
loss = loss + mu(3) * (s_23 - s_12)^2 + mu(4) * norm(R_13 - R_12 * R_23, 2) + mu(5) * norm(t_13 - R_12 * t_23 - t_12, 2);
end