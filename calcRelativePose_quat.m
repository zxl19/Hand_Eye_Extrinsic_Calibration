function [R_ab, t_ab] = calcRelativePose_quat(pose_a, pose_b)
R_a = quat2rotm(pose_a(1, 4 : 7));
t_a = pose_a(1, 1 : 3)';
R_b = quat2rotm(pose_b(1, 4 : 7));
t_b = pose_b(1, 1 : 3)';
R_ab = R_a \ R_b; % Important!!!
t_ab = R_a \ (t_b - t_a); % Important!!!
end