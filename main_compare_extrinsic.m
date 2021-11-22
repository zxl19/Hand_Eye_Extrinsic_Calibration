close all
clear
clc
%% Parameter Setup
filename_x_LI = "./results/2021-11-10/bag1/x_LI.mat"; % LiDAR to INS Extrinsic
filename_x_CI = "./results/2021-11-10/bag1/x_CI.mat"; % Camera to INS Extrinsic
filename_x_LC = "./results/2021-11-10/bag1/x_LC.mat"; % LiDAR to Camera Extrinsic
filename_x_LI_FCPE = "./results/2021-11-10/bag1/x_LI_FCPE.mat"; % LiDAR to INS Extrinsic
filename_x_CI_FCPE = "./results/2021-11-10/bag1/x_CI_FCPE.mat"; % Camera to INS Extrinsic
filename_x_LC_FCPE = "./results/2021-11-10/bag1/x_LC_FCPE.mat"; % LiDAR to Camera Extrinsic
filename_x_LI_FCPE_update = "./results/2021-11-10/bag1/x_LI_FCPE_update.mat"; % LiDAR to INS Extrinsic
filename_x_CI_FCPE_update = "./results/2021-11-10/bag1/x_CI_FCPE_update.mat"; % Camera to INS Extrinsic
filename_x_LC_FCPE_update = "./results/2021-11-10/bag1/x_LC_FCPE_update.mat"; % LiDAR to Camera Extrinsic
%% Load Extrinsic
data_LI = load(filename_x_LI, '-ascii');
data_CI = load(filename_x_CI, '-ascii');
data_LC = load(filename_x_LC, '-ascii');
data_LI_FCPE = load(filename_x_LI_FCPE, '-ascii');
data_CI_FCPE = load(filename_x_CI_FCPE, '-ascii');
data_LC_FCPE = load(filename_x_LC_FCPE, '-ascii');
data_LI_FCPE_update = load(filename_x_LI_FCPE_update, '-ascii');
data_CI_FCPE_update = load(filename_x_CI_FCPE_update, '-ascii');
data_LC_FCPE_update = load(filename_x_LC_FCPE_update, '-ascii');
%%% Hand-Eye Calibration
t_LI = data_LI(1, 1 : 3);
q_LI = data_LI(1, 4 : 7);
T_LI = quat2tform(q_LI);
T_LI(1 : 3, 4) = t_LI';
t_CI = data_CI(1, 1 : 3);
q_CI = data_CI(1, 4 : 7);
scale_CI = data_CI(1, 8);
T_CI = quat2tform(q_LI);
T_CI(1 : 3, 4) = t_CI';
t_LC = data_LC(1, 1 : 3);
q_LC = data_LC(1, 4 : 7);
scale_LC = data_LC(1, 8);
T_LC = quat2tform(q_LC);
T_LC(1 : 3, 4) = t_LC';
%%% Round 1
t_LI_FCPE = data_LI_FCPE(1, 1 : 3);
q_LI_FCPE = data_LI_FCPE(1, 4 : 7);
T_LI_FCPE = quat2tform(q_LI_FCPE);
T_LI_FCPE(1 : 3, 4) = t_LI_FCPE';
t_CI_FCPE = data_CI_FCPE(1, 1 : 3);
q_CI_FCPE = data_CI_FCPE(1, 4 : 7);
scale_CI_FCPE = data_CI_FCPE(1, 8);
T_CI_FCPE = quat2tform(q_LI_FCPE);
T_CI_FCPE(1 : 3, 4) = t_CI_FCPE';
t_LC_FCPE = data_LC_FCPE(1, 1 : 3);
q_LC_FCPE = data_LC_FCPE(1, 4 : 7);
scale_LC_FCPE = data_LC_FCPE(1, 8);
T_LC_FCPE = quat2tform(q_LC_FCPE);
T_LC_FCPE(1 : 3, 4) = t_LC_FCPE';
%%% Round 2
t_LI_FCPE_update = data_LI_FCPE_update(1, 1 : 3);
q_LI_FCPE_update = data_LI_FCPE_update(1, 4 : 7);
T_LI_FCPE_update = quat2tform(q_LI_FCPE_update);
T_LI_FCPE_update(1 : 3, 4) = t_LI_FCPE_update';
t_CI_FCPE_update = data_CI_FCPE_update(1, 1 : 3);
q_CI_FCPE_update = data_CI_FCPE_update(1, 4 : 7);
scale_CI_FCPE_update = data_CI_FCPE_update(1, 8);
T_CI_FCPE_update = quat2tform(q_LI_FCPE_update);
T_CI_FCPE_update(1 : 3, 4) = t_CI_FCPE_update';
t_LC_FCPE_update = data_LC_FCPE_update(1, 1 : 3);
q_LC_FCPE_update = data_LC_FCPE_update(1, 4 : 7);
scale_LC_FCPE_update = data_LC_FCPE_update(1, 8);
T_LC_FCPE_update = quat2tform(q_LC_FCPE_update);
T_LC_FCPE_update(1 : 3, 4) = t_LC_FCPE_update';
%% Display Results
consistence_HE = norm(T_LC * T_CI - T_LI, 2);
consistence_FCPE = norm(T_LC_FCPE * T_CI_FCPE - T_LI_FCPE, 2);
consistence_update = norm(T_LC_FCPE_update * T_CI_FCPE_update - T_LI_FCPE_update, 2);
fprintf("Hand-Eye Calibration Consistence:\t\t%.6f\n", consistence_HE)
fprintf("Round 1 Joint Calibration Consistence:\t%.6f\n", consistence_FCPE)
fprintf("Round 2 Joint Calibration Consistence:\t%.6f\n", consistence_update)