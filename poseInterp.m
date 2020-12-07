function [pose_1_interp, timestamp_1_interp, pose_2_interp, timestamp_2_interp] = poseInterp(pose_1, timestamp_1, pose_2, timestamp_2)
[n1, ~] = size(pose_1);
[n2, ~] = size(pose_2);
ind = 1;
for i = 1 : n1
    [index, ~] = findNearest(timestamp_1(i), timestamp_2);
    if (timestamp_1(i) < timestamp_2(index))
        if (index == 1)
            continue
        end
        T = (timestamp_1(i) - timestamp_2(index - 1)) / (timestamp_2(index) - timestamp_2(index - 1));
%         lb_trans = pose_2(index - 1, 1 : 3);
%         ub_trans = pose_2(index, 1 : 3);
        lb_quat = quaternion(pose_2(index - 1, 4 : 7));
        ub_quat = quaternion(pose_2(index, 4 : 7));
    elseif (timestamp_1(i) > timestamp_2(index))
        if (index == n2)
            continue
        end
        T = (timestamp_1(i) - timestamp_2(index)) / (timestamp_2(index + 1) - timestamp_2(index));
%         lb_trans = pose_2(index, 1 : 3);
%         ub_trans = pose_2(index + 1, 1 : 3);
        lb_quat = quaternion(pose_2(index, 4 : 7));
        ub_quat = quaternion(pose_2(index + 1, 4 : 7));
    end
    timestamp_1_interp(ind, 1) = timestamp_1(i);
    timestamp_2_interp(ind, 1) = timestamp_1(i);
    pose_1_interp(ind, :) = pose_1(i, :);
%     trans = lb_trans * T + ub_trans * (1 - T); % Naive Linear Interpolation
    % Linear Interpolation
%     x = interp1(timestamp_2, pose_2(:, 1), timestamp_1(i), 'linear');
%     y = interp1(timestamp_2, pose_2(:, 2), timestamp_1(i), 'linear');
%     z = interp1(timestamp_2, pose_2(:, 3), timestamp_1(i), 'linear');
    % Cubic Interpolation
    x = interp1(timestamp_2, pose_2(:, 1), timestamp_1(i), 'pchip');
    y = interp1(timestamp_2, pose_2(:, 2), timestamp_1(i), 'pchip');
    z = interp1(timestamp_2, pose_2(:, 3), timestamp_1(i), 'pchip');
    trans = [x, y, z];
    [qw, qx, qy, qz] = parts(slerp(lb_quat, ub_quat, T));
    pose_2_interp(ind, :) = [trans, qw, qx, qy, qz];
    ind = ind + 1;
end