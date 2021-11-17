close all
clear
clc
format long
%% Filename Setup
input_filename = "../raw_data/calib_2020-11-15-11-54-47-aft_mapped_to_init.csv";
output_filename = "../data/LO.mat";
% input_filename = "../raw_data/calib_2020-12-02-17-04-12-aft_mapped_to_init.csv";
% output_filename = "../data/LO_3.mat";
%% Read Data
T = readtable(input_filename);
timestamp = T{:, 1} * 1e-9; % s
pose = T{:, [7 : 9, 13, 10 : 12]}; % x y z qw qx qy qz
data = [timestamp, pose];
%% Sort Data Accoring to Timestamp
data = sortrows(data);
%% Output Formatted Pose
% writematrix([timestamp, pose], output_filename, 'Delimiter', ' '); % Do not use, causes numerical error!
save(output_filename, 'data', '-ascii', '-double');