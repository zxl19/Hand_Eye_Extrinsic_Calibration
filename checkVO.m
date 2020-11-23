close all
clear
clc
%%
filename_1 = "pose5.txt";
%%
T = readtable(filename_1);
% pose = T{:, 6 : 8};
pose = T{:, 2 : 4};
%%
figure
plot3(pose(:, 1), pose(:, 2), pose(:, 3), 'bs', 'LineWidth', 1)
axis equal
view(3)