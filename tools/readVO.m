close all
clear
clc
format long
%% Filename Setup
input_filename = "../raw_data/pose6.txt";
output_filename = "../data/VO.mat";
%% Read Data
T = readtable(input_filename);
timestamp = T{:, 1}; % s
pose = T{:, 2 : 8}; % x y z qw qx qy qz
data = [timestamp, pose];
%% Sort Data Accoring to Timestamp
data = sortrows(data);
%% Output Formatted Pose
% writematrix([timestamp, pose], output_filename, 'Delimiter', ' '); % Do not use, causes numerical error!
save(output_filename, 'data', '-ascii', '-double');