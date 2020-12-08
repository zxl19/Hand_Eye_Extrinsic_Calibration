close all
clear
clc
format long
%% README
% Convert formatted pose file to .mat format.
%% Filename Setup
input_filename = "../raw_data/xxx.csv"; % Change input pose filename here.
output_filename = "../data/xxx.mat";    % Change output .mat filename here.
%% Read Data
T = readtable(input_filename);
timestamp = T{:, 1}; % s
pose = T{:, 2 : 8}; % x y z qw qx qy qz
data = [timestamp, pose];
%% Output Formatted Pose
% writematrix([timestamp, pose], output_filename, 'Delimiter', ' '); % Do not use, causes numerical error!
save(output_filename, 'data', '-ascii', '-double');