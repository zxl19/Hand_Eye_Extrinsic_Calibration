close all
clear
clc
format long
%% Filename Setup
input_filename = "../raw_data/8zi_biaoding-imu-data.csv";
output_filename = "../data/IMU.mat";
%% Read Data
T = readtable(input_filename);
timestamp = T{:, 3} + T{:, 4} * 1e-9; % s
%% Sort Data Accoring to Timestamp
data = sortrows(data);
%% Output Formatted Pose
% writematrix(timestamp, output_filename); % Do not use, causes numerical error!
save(output_filename, 'timestamp', '-ascii', '-double');