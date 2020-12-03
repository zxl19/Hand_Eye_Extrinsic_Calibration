# Hand_Eye_Extrinsic_Calibration

**This repository is currently in progress.**

MATLAB code for LiDAR-GPS/IMU and Camera-GPS/IMU extrinsic calibration based on hand-eye calibration method.

The rotation part of the extrinsic can be represented by euler angles, quaternions or 9 elements of the rotation matrix. We have implemented those 3 representations respectively by optimizing `x y z yaw pitch roll` or `x y z qw qx qy qz` or `x y z r_11 r_12 r_13 r_21 r_22 r_23 r_31 r_32 r_33`[^1].

## Data Preparation

1. Drive the vehicle in a "$\infty$" shaped trajectory.
2. Record LiDAR, camera, GPS/IMU and IMU topics.

    ```shell
    rosbag record -o calib /velodyne_points /usb_cam_left/image_raw/compressed /novatel_data/inspvax /imu/data
    ```

## LiDAR to GPS/IMU Extrinsic Calibration

### LiDAR Pose Estimation

1. Record pose estimates of a SLAM algorithm (e.g. [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)) and GPS/IMU pose output.

    ```shell
    rosbag record -o out /aft_mapped_to_init /novatel_data/inspvax
    ```

2. Convert rosbag to `.csv` format.
    - Use [rosbag_to_csv](https://github.com/AtsushiSakai/rosbag_to_csv).
        - Change `line 30` and `line 42` in `/scripts/rosbag_to_csv.py`
            from:

            ```python
            stream.write("," + msg_str)             # line 30
            stream.write("," + parent_content_name) # line 42
            ```

            to:

            ```python
            stream.write(", " + msg_str)             # line 30
            stream.write(", " + parent_content_name) # line 42
            ```

        - Change `line 81` in `/scripts/rosbag_to_csv.py`
            from:

            ```python
            stream.write(datetime.fromtimestamp(time.to_time()).strftime('%Y/%m/%d/%H:%M:%S.%f'))
            ```

            to:

            ```python
            stream.write(str(time))
            ```

    - Or just use [my fork](https://github.com/zxl19/rosbag_to_csv).

### Calibration

1. Change filenames of `.csv` files.

    ```matlab
    %% Pose Filename Setup
    filename_1 = "pose1.csv"; % LiDAR Odometry
    filename_2 = "pose2.csv"; % GPS/IMU
    ```

2. Run `main_calibration_L2I_*.m`

## Camera to GPS/IMU

### Camera Pose Estimation

### Calibration

1. Change filenames of `.csv` and `.txt` files.

    ```matlab
    %% Pose Filename Setup
    filename_1 = "pose1.csv"; % Visual Odometry
    filename_2 = "pose2.txt"; % GPS/IMU
    filename_3 = "imu_time.csv"; % IMU Time
    ```

2. Run `main_calibration_C2I_*.m`

## Reference

1. [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)
2. [rosbag_to_csv](https://github.com/AtsushiSakai/rosbag_to_csv)
3. [MATLAB-GPS-Calculations](https://github.com/alexbuczynsky/MATLAB-GPS-Calculations)
4. [COLMAP](https://github.com/colmap/colmap)

[^1]: Dornaika F, Horaud R. Simultaneous Robot-World and Hand-Eye Calibration[J]. IEEE Trans Robotics Automat, 1998, 14(4):617-622. [[LINK](https://ieeexplore.ieee.org/document/704233)]
