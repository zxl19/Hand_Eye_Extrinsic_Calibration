function [timestamp, pose] = readVO(filename)
T = readtable(filename);
timestamp = T{:, 1}; % s
pose = T{:, 2 : 8}; % x y z qw qx qy qz
end