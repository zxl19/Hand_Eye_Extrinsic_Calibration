# Hand_Eye_Extrinsic_Calibration

**This repository is currently in progress.**

MATLAB code for LiDAR-INS and Camera-INS extrinsic calibration based on hand-eye calibration method.

The rotation part of the extrinsic can be represented by euler angles, quaternions or 9 elements of the rotation matrix. We have implemented those 3 representations respectively by optimizing `x y z yaw pitch roll` or `x y z qw qx qy qz` or `x y z r_11 r_12 r_13 r_21 r_22 r_23 r_31 r_32 r_33`[^1].

## Prerequisites

1. Ubuntu (tested on 16.04) and ROS (tested on Kinetic).
2. MATLAB (tested on 2020a, with Robotics System Toolbox installed).

## Data Preparation

1. Drive the vehicle in a "$\infty$" shaped trajectory.
2. Record LiDAR, camera, INS and IMU topics.

    ```shell
    rosbag record -o calib /velodyne_points /usb_cam_left/image_raw/compressed /novatel_data/inspvax /imu/data
    ```

## LiDAR to INS Extrinsic Calibration

### LiDAR Pose Estimation

1. Record pose estimates of a SLAM algorithm (e.g. [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)) and INS pose output.

    ```shell
    rosbag record -o out /aft_mapped_to_init /novatel_data/inspvax
    ```

2. Convert the recorded topics into `.csv` format.
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
    filename_2 = "pose2.csv"; % INS
    ```

2. Run `main_calibration_L2I_*.m`
    - eul: Use euler angles to represent rotation.
    - quat: Use quaternions to represent rotation.
    - 12: Use 12 elements of the rotation matrix to represent rotation.
    - quat_interp: Use quaternions to represent rotation. Instead of synchronizing timestamps, we use cubic interpolation to smooth translation and spherical linear interpolation (SLERP) to smooth rotation. (recommended)

## Camera to INS Extrinsic Calibration

### Camera Pose Estimation

1. We use [COLMAP](https://github.com/colmap/colmap) to estimate and export camera pose. The `.txt` pose output should be in the following format:

    ```text
    timestamp x y z qw qx qy qz
    ```

    where `timestamp` is the original timestamp when the data is recorded.

2. Convert IMU topic into `.csv` format. We use IMU timestamp as the original INS timestamp.

### Calibration

1. Change filenames of `.csv` and `.txt` files.

    ```matlab
    %% Pose Filename Setup
    filename_1 = "pose1.csv"; % Visual Odometry
    filename_2 = "pose2.txt"; % INS
    filename_3 = "imu_time.csv"; % IMU Time
    ```

2. Run `main_calibration_C2I_*.m`
    - eul: Use euler angles to represent rotation.
    - quat: Use quaternions to represent rotation.
    - 12: Use 12 elements of the rotation matrix to represent rotation.
    - quat_interp: Use quaternions to represent rotation. Instead of synchronizing timestamps, we use cubic interpolation to smooth translation and spherical linear interpolation (SLERP) to smooth rotation. (recommended)

## Reference

1. [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)
2. [rosbag_to_csv](https://github.com/AtsushiSakai/rosbag_to_csv)
3. [COLMAP](https://github.com/colmap/colmap)
4. [MATLAB-GPS-Calculations](https://github.com/alexbuczynsky/MATLAB-GPS-Calculations)

[^1]: Dornaika F, Horaud R. Simultaneous Robot-World and Hand-Eye Calibration[J]. IEEE Trans Robotics Automat, 1998, 14(4):617-622. [[LINK](https://ieeexplore.ieee.org/document/704233)]
