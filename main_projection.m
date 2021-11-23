close all
clear
clc
format
%% 1 Parameter Setup
%% 1.1 Camera Intrinsic
fx = 1052.5378;
fy = 1052.5378;
cx = 985.2623;
cy = 551.7450;
%% 1.2 LiDAR to Camera Extrinsic
%% 1.2.1 Hand Measured Extrinsic
% eul = [pi / 2, 0, -pi / 2]; % ZYX
t_LC = [-0.5, 0.06, 0.0]; % x y z Hand Measured
% q_LC = eul2quat(eul, 'ZYX');
%% 1.2.2 bag1
%% a. Hand-Eye Calibration
% t_LC = [-0.4450, -0.0657, -0.0547]; % x y z
% q_LC = [-0.5031, 0.5092, 0.4867, -0.5008]; % qw qx qy qz
%% b. FCPE
% t_LC = [-0.3787, -0.0691, -0.2201]; % x y z
% q_LC = [0.5039, -0.5082, -0.4873, 0.5002]; % qw qx qy qz
%% c. FCPE update
q_LC = [0.5043, -0.5088, -0.4867, 0.4999]; % qw qx qy qz
%% 1.2.3 bag2
%% a. Hand-Eye Calibration
% t_LC = [-0.5024, -0.0142, -0.1498]; % x y z
% q_LC = [0.5038, -0.5103, -0.4856, 0.4999]; % qw qx qy qz
%% b. FCPE
% t_LC = [-0.4022, -0.0144, -0.1999]; % x y z
% q_LC = [0.5035, -0.5108, -0.4848, 0.5005]; % qw qx qy qz
%% c. FCPE update
% q_LC = [0.5044, -0.5103, -0.4856, 0.4994]; % qw qx qy qz
%% 1.2.4 bag3
%% a. Hand-Eye Calibration
% t_LC = [-0.4693, 0.0236, -0.1446]; % x y z
% q_LC = [-0.5046, 0.5102, 0.4845, -0.5003]; % qw qx qy qz
%% b. FCPE
% t_LC = [-0.3966, 0.0184, -0.2016]; % x y z
% q_LC = [0.5052, -0.5095, -0.4857, 0.4993]; % qw qx qy qz
%% c. FCPE update
% q_LC = [0.5047, -0.5107, -0.4845, 0.4997]; % qw qx qy qz
%% 1.2.4 bag4
%% a. Hand-Eye Calibration
% t_LC = [-0.4418, 0.0191, -0.0411]; % x y z
% q_LC = [0.5055, -0.5110, -0.4841, 0.4991]; % qw qx qy qz
%% b. FCPE
% t_LC = [-0.3888, 0.0162, -0.2122]; % x y z
% q_LC = [0.5066, -0.5099, -0.4854, 0.4977]; % qw qx qy qz
%% c. FCPE update
% q_LC = [0.5051, -0.5119, -0.4837, 0.4987]; % qw qx qy qz
%% 1.2.5 bag6
%% a. Hand-Eye Calibration
% t_LC = [-0.4784, -0.0196, -0.0759]; % x y z
% q_LC = [-0.5043, 0.5090, 0.4863, -0.5001]; % qw qx qy qz
%% b. FCPE
% t_LC = [-0.3870, -0.0175, -0.1360]; % x y z
% q_LC = [0.5051, -0.5080, -0.4872, 0.4995]; % qw qx qy qz
%% c. FCPE update
% q_LC = [0.5062, -0.5077, -0.4874, 0.4985]; % qw qx qy qz
%% 1.2.6 bag7
%% a. Hand-Eye Calibration
% t_LC = [-0.4490, -0.0331, -0.1428]; % x y z
% q_LC = [0.5044, -0.5089, -0.4862, 0.5003]; % qw qx qy qz
%% b. FCPE
% t_LC = [-0.3851, -0.0411, -0.0538]; % x y z
% q_LC = [0.5034, -0.5105, -0.4848, 0.5009]; % qw qx qy qz
%% c. FCPE update
% q_LC = [0.5046, -0.5098, -0.4854, 0.4999]; % qw qx qy qz
%%%
R_LC = quat2rotm(q_LC);
%% 1.3 LiDAR Point Cloud and Image Filename
filename_pcd = "./raw_data/2021-11-10/static1/static1.pcd";
filename_img = "./raw_data/2021-11-10/static1/static1.jpg";
filename_out = "./results/2021-11-10/bag1/static1_projected.png";
% filename_pcd = "./raw_data/2021-11-10/static2/static2.pcd";
% filename_img = "./raw_data/2021-11-10/static2/static2.jpg";
% filename_out = "./results/2021-11-10/bag1/static2_projected.png";
% Static 3 is the same as Static 1, therefore is removed.
% Static 4 has dynamic objects, therefore is removed.
% filename_pcd = "./raw_data/2021-11-10/static5/static5.pcd";
% filename_img = "./raw_data/2021-11-10/static5/static5.jpg";
% filename_out = "./results/2021-11-10/bag1/static5_projected.png";
% filename_pcd = "./raw_data/2021-11-10/static6/static6.pcd";
% filename_img = "./raw_data/2021-11-10/static6/static6.jpg";
% filename_out = "./results/2021-11-10/bag1/static6_projected.png";
% Garage 1 to 5 has other objects, therefore is removed.
% filename_pcd = "./raw_data/2021-11-19/garage6/garage6.pcd";
% filename_img = "./raw_data/2021-11-19/garage6/garage6.jpg";
% filename_out = "./results/2021-11-10/bag1/garage6_projected.png";
%% Read and Display LiDAR Point Cloud
ptCloud = pcread(filename_pcd);
figure
pcshow(ptCloud)
hold on
xlabel('X / m')
ylabel('Y / m')
zlabel('Z / m')
%% Project LiDAR Points onto Image
Image = imread(filename_img);
[imgHeight, imgWidth, ~] = size(Image);
P_L = ptCloud.Location;
[N, ~] = size(P_L);
ind = 1;
for i = 1 : N
    temp = R_LC' * P_L(i, :)' - R_LC' * t_LC';
    P_L2C(i, :) = temp';
    if temp(3) > 0
        u(ind, :) = (fx * temp(1) + cx * temp(3)) / temp(3);
        v(ind, :) = (fy * temp(2) + cy * temp(3)) / temp(3);
        distance(ind, :) = sqrt(sum(temp.^2));
        ind = ind + 1;
    end
end
u = round(u);
v = round(v);
ImgROI = (u > 0 & u < imgWidth) & (v > 0 & v < imgHeight);
u = u(ImgROI);
v = v(ImgROI);
distance = distance(ImgROI);
[M, ~] = size(u);
%%% Colormaps
% c = jet;
c = hsv; % Looks Better
% c = colorcube;
for i = 1 : M
    ratio = (distance(i) - min(distance)) / (max(distance) - min(distance));
    index = round(ratio * 255) + 1;
    color = 255 * c(index, :);
    Image(v(i), u(i), :) = color;
    if (v(i) > 1) && (v(i) < imgWidth)
        Image(v(i) + 1, u(i), :) = color;
        Image(v(i) - 1, u(i), :) = color;
    end
    if (u(i) > 1) && (u(i) < imgWidth)
        Image(v(i), u(i) + 1, :) = color;
        Image(v(i), u(i) - 1, :) = color;
    end
end
%% Display Projection Result
figure
pcshow(pointCloud(P_L2C))
hold on
xlabel('X / m')
ylabel('Y / m')
zlabel('Z / m')
figure
plot(u, v, 'b.')
axis equal
xlabel('u / px')
ylabel('v / px')
set(gca,'YDir','reverse');
figure
hold on
imshow(Image)
%% Crop
Image2 = imcrop(Image, [1, 450, 1920, 450]);
figure
imshow(Image2)
%% Save Projection Result
imwrite(Image2, filename_out);
% imwrite(getframe(gcf).cdata, 'test.png');
% imwrite(getframe(gca).cdata, 'test.png');