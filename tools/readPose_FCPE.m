close all
clear
clc
format long
%% Filename Setup
input_filename_LO = "../raw_data/calib_2020-11-15-11-54-47-aft_mapped_to_init.csv";
input_filename_VO = "../raw_data/pose6.txt";
input_filename_INS = "../raw_data/calib_2020-11-15-11-54-47-novatel_data-inspvax.csv";
input_filename_IMU = "../raw_data/8zi_biaoding-imu-data.csv";
output_filename_LO = "../data/LO_FCPE.mat";
output_filename_VO = "../data/VO_FCPE.mat";
output_filename_INS = "../data/INS_FCPE.mat";
% input_filename_LO = "../raw_data/2021-11-10/bag1/bag1-aft_mapped_to_init.csv";
% input_filename_VO = "../raw_data/2021-11-10/bag1/bag1-images.txt";
% input_filename_INS = "../raw_data/2021-11-10/bag1/bag1-novatel_data-inspvax.csv";
% input_filename_IMU = "../raw_data/2021-11-10/bag7/bag1-imu-data.csv";
% output_filename_LO = "../data/2021-11-10/bag1/LO_FCPE.mat";
% output_filename_VO = "../data/2021-11-10/bag1/VO_FCPE.mat";
% output_filename_INS = "../data/2021-11-10/bag1/INS_FCPE.mat";
%% Read LO Data
T_LO = readtable(input_filename_LO);
timestamp_LO = T_LO{:, 3} + T_LO{:, 4} * 1e-9; % s
pose_LO = T_LO{:, [7 : 9, 13, 10 : 12]}; % x y z qw qx qy qz
data_LO = [timestamp_LO, pose_LO];
%% Read VO Data
T_VO = readtable(input_filename_VO);
timestamp_VO = T_VO{:, 1}; % s
pose_VO = T_VO{:, 2 : 8}; % x y z qw qx qy qz
data_VO = [timestamp_VO, pose_VO];
%% Read IMU Data
T_IMU = readtable(input_filename_IMU);
timestamp_IMU = T_IMU{:, 3} + T_IMU{:, 4} * 1e-9; % s
%% Read INS Data
T_INS = readtable(input_filename_INS);
timestamp_INS = timestamp_IMU; % s
latitude = T_INS{:, 16}; % degree
longitude = T_INS{:, 17}; % degree
altitude = T_INS{:, 18}; % m
roll = deg2rad(T_INS{:, 23}); % rad
pitch = deg2rad(T_INS{:, 24}); % rad
azimuth = deg2rad(T_INS{:, 25}); % rad
yaw = azi2yaw(azimuth); % rad
%% Convert Data Format
[x, y, ~] = deg2utm(latitude, longitude); % degree
z = altitude;
% Follow Novatel Driver Convention
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
pose_INS = [x, y, z, quat]; % x y z qw qx qy qz
data_INS = [timestamp_INS, pose_INS];
%% Sort Data Accoring to Timestamp
data_LO = sortrows(data_LO);
data_VO = sortrows(data_VO);
data_INS = sortrows(data_INS);
%% Output Formatted Pose
% writematrix([timestamp, pose], output_filename, 'Delimiter', ' '); % Do not use, causes numerical error!
save(output_filename_LO, 'data_LO', '-ascii', '-double');
save(output_filename_VO, 'data_VO', '-ascii', '-double');
save(output_filename_INS, 'data_INS', '-ascii', '-double');