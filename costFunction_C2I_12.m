function loss = costFunction_C2I_12(pose_1, pose_2, x0)
mu = [1, 1, 1e6, 1e6]; % weight
R = [x0(1, 1 : 3); x0(1, 4 : 6); x0(1, 7 : 9)];
t = x0(1, 10 : 12)';
s = x0(1, 13); % scale
loss = 0;
[n, ~] = size(pose_1);
for i = 2 : n
    [R_1, t_1] = calcRelativePose_quat(pose_1(i - 1, :), pose_1(i, :));
    [R_2, t_2] = calcRelativePose_quat(pose_2(i - 1, :), pose_2(i, :));
%     loss = loss + mu(1) * norm(R_1 * R - R * R_2, 2) + ...
%         mu(2) * norm(R_1 * t + t_1 * s - R * t_2 - t, 2); % Worse
%     loss = loss + mu(1) * norm(R_1 * R - R * R_2, 2) + ...
%         mu(2) * norm(R_1 * t + t_1 * s - R * t_2 - t, 2) + ...
%         mu(2) * norm(x0(1, 1 : 3) - [0.3, 0.11, -0.85], 2); % Consider Measurement
    loss = loss + mu(1) * (norm(R_1 * R - R * R_2, 2))^2 + ...
        mu(2) * (norm(R_1 * t + t_1 * s - R * t_2 - t, 2))^2;
end
% loss = loss + mu(3) * norm(R * R' - eye(3), 2) + mu(4) * norm(R * R' - eye(3), 2);
loss = loss + mu(3) * norm(R * R' - eye(3), 2)^2 + mu(4) * norm(R * R' - eye(3), 2)^2;
end