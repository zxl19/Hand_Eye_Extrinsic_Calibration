close all
clear
clc
format long
%% Filename Setup
input_filename = "../raw_data/calib_2020-11-15-11-54-47-novatel_data-inspvax.csv";
output_filename = "../data/INS.mat";
% input_filename = "../raw_data/calib_2020-12-02-17-04-12-novatel_data-inspvax.csv";
% output_filename = "../data/INS_3.mat";
%% Read Data
T = readtable(input_filename);
timestamp = T{:, 1} * 1e-9; % s
latitude = T{:, 16}; % degree
longitude = T{:, 17}; % degree
altitude = T{:, 18}; % m
roll = deg2rad(T{:, 23}); % rad
pitch = deg2rad(T{:, 24}); % rad
azimuth = deg2rad(T{:, 25}); % rad
yaw = azi2yaw(azimuth); % rad
%% Convert Data Format
[x, y, ~] = deg2utm(latitude, longitude); % degree
z = altitude;
% Work
eul = [-azimuth, pitch, roll]; % yaw pitch roll
quat = eul2quat(eul, 'ZYX'); % qw qx qy qz
% Novatel ZXY
% Not Working
% zero = zeros(size(roll));
% eul_1 = [yaw, zero, zero]; % yaw roll(0) pitch(0)
% eul_2 = [roll, pitch, zero]; % roll pitch yaw(0)
% rotm_1 = eul2rotm(eul_1, 'ZYX'); % Z00
% rotm_2 = eul2rotm(eul_2, 'XYZ'); % XY0
% rotm = rotm_1 .* rotm_2; % ZXY
% quat = rotm2quat(rotm); % qw qx qy qz
pose = [x, y, z, quat]; % x y z qw qx qy qz
data = [timestamp, pose];
%% Output Formatted Pose
% writematrix([timestamp, pose], output_filename, 'Delimiter', ' '); % Do not use, causes numerical error!
save(output_filename, 'data', '-ascii', '-double');