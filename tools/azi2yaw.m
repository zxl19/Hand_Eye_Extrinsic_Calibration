function yaw = azi2yaw(azimuth)
[len, ~] = size(azimuth);
yaw = zeros(len, 1);
%% yaw [0, 2 * pi)
for i = 1 : len
    if azimuth(i) <= pi / 2
        yaw(i) = pi / 2 - azimuth(i);
    elseif azimuth(i) > pi / 2
        yaw(i) = 2 * pi - (azimuth(i) - pi / 2);
    end
end
%% yaw (-pi, pi]
% for i = 1 : len
%     if azimuth(i) < 3 * pi / 2
%         yaw(i) = pi / 2 - azimuth(i);
%     elseif azimuth(i) >= 3 * pi / 2
%         yaw(i) = 5 * pi / 2 - azimuth(i);
%     end
% end
end