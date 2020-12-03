function [timestamp, pose] = readNovatel(filename)
T = readtable(filename);
timestamp = T{:, 1} * 1e-9; % s
latitude = T{:, 16};
longitude = T{:, 17};
altitude = T{:, 18};
roll = deg2rad(T{:, 23}); % rad
pitch = deg2rad(T{:, 24}); % rad
azimuth = deg2rad(T{:, 25}); % rad
pose = [latitude, longitude, altitude, -azimuth, pitch, roll]; % latitude longitude altitude yaw pitch roll
end