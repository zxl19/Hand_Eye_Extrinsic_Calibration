function [R_ab, t_ab] = calcRelativePose(pose_a, pose_b)
R_a = eul2rotm(pose_a(1, 4 : 6), 'XYZ');
t_a(1 : 3, 4) = pose_a(1, 1 : 3)';
R_b = eul2rotm(pose_b(1, 4 : 6), 'XYZ');
t_b(1 : 3, 4) = pose_b(1, 1 : 3)';
R_ab = R_a \ R_b;
t_ab = t_b - t_a;
end