close all
clear
clc
format
%% T
T = [-1.8193e-02, 3.7617e-02, -9.9913e-01, 1.035e-01; ...
    9.9983e-01, -1.5235e-03, -1.8263e-02, -4.3383e-01; ...
    -2.2092e-03, -9.9929e-01, -3.7583e-02, 4.5814e-01; ...
    0, 0, 0, 1]
T(1: 3, 1 : 3)
%% 
% eul = tform2eul(T, 'ZYX')
eul = rotm2eul(T(1: 3, 1 : 3), 'ZYX')
% eul = [0 pi/2 0]
% eul = [pi/4 0 -pi/4]
rotm1 = eul2rotm(eul, 'ZYX')
rotm2 = myEul2Rotm(eul, 'ZYX')
rotm3 = eul2rotm(eul(3 : -1 : 1), 'XYZ')
rotm4 = myEul2Rotm(eul(3 : -1 : 1), 'XYZ')
