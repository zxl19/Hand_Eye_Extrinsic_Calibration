function R = myEul2Rotm(eulerAngle, s)
roll = eulerAngle(1);
pitch = eulerAngle(2);
yaw = eulerAngle(3);
R = zeros(3);
R_x = [1, 0, 0; ...
    0, cos(roll), -sin(roll); ...
    0, sin(roll), cos(roll)];
R_y = [cos(pitch), 0, sin(pitch); ...
    0, 1, 0;
    -sin(pitch), 0, cos(pitch)];
R_z = [cos(yaw), -sin(yaw), 0; ...
    sin(yaw), cos(yaw), 0; ...
    0, 0, 1];
ct = cos(eulerAngle);
st = sin(eulerAngle);
if s == 'XYZ'
    R = R_x * R_y * R_z;
elseif s == 'ZYX'
%     R = R_z * R_y * R_x;
    R(1,1) = ct(2).*ct(1);
    R(1,2) = st(3).*st(2).*ct(1) - ct(3).*st(1);
    R(1,3) = ct(3).*st(2).*ct(1) + st(3).*st(1);
    R(2,1) = ct(2).*st(1);
    R(2,2) = st(3).*st(2).*st(1) + ct(3).*ct(1);
    R(2,3) = ct(3).*st(2).*st(1) - st(3).*ct(1);
    R(3,1) = -st(2);
    R(3,2) = st(3).*ct(2);
    R(3,3) = ct(3).*ct(2);
end
end