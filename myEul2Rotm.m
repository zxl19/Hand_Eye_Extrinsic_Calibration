function R = myEul2Rotm(eulerAngle, s)
roll = eulerAngle(1);
pitch = eulerAngle(2);
yaw = eulerAngle(3);
R_x = [1, 0, 0; ...
    0, cos(roll), -sin(roll); ...
    0, sin(roll), cos(roll)];
R_y = [cos(pitch), 0, sin(pitch); ...
    0, 1, 0;
    -sin(pitch), 0, cos(pitch)];
R_z = [cos(yaw), -sin(yaw), 0; ...
    sin(yaw), cos(yaw), 0; ...
    0, 0, 1];
if s == 'XYZ'
    R = R_x * R_y * R_z;
elseif s == 'ZYX'
    R = R_z * R_y * R_x;
end
end