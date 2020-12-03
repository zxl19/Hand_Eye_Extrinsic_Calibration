function [timestamp, pose] = readLO(filename)
T = readtable(filename);
timestamp = T{:, 1} * 1e-9; % s
pose = T{:, [7 : 9, 13, 10 : 12]}; % x y z qw qx qy qz
end