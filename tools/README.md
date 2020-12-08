# Data Preparation and Format Conversion

If you have prepared your data in the following format:

```text
timestamp x y z qw qx qy qz
```

Set your filenames and run `pose2mat.m` to get `.mat` files.

------

**ELSE......**

## 1. Data Preparation

1. Drive the vehicle in a "$\infty$" shaped trajectory.
2. Record LiDAR, camera, INS and IMU topics.

    ```shell
    rosbag record -o calib /velodyne_points /usb_cam_left/image_raw/compressed /novatel_data/inspvax /imu/data
    ```

## 2. Pose Estimation

### 2.1 LiDAR Pose Estimation

1. Record pose estimates of a SLAM algorithm (e.g. [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)) and INS pose output.

    ```shell
    rosbag record -o out /aft_mapped_to_init /novatel_data/inspvax
    ```

2. Convert the recorded topics into `.csv` format using `rosbag_to_csv`.
3. Convert `.csv` files to `.mat` files.
    - `readLO`: Convert LiDAR odometry file to `.mat` format.
    - `readNovatel`: Convert Novatel pose file to `.mat` format.

### 2.2 Camera Pose Estimation

1. We use [COLMAP](https://github.com/colmap/colmap) to estimate and export camera pose. The `.txt` pose output should be in the following format:

    ```text
    timestamp x y z qw qx qy qz
    ```

    where `timestamp` is the original timestamp when the image data is recorded.

2. Convert INS and IMU topics into `.csv` format using `rosbag_to_csv`. Here we use IMU timestamp as the original INS timestamp.
3. Convert `.csv` files to `.mat` files.
    - `readVO`: Convert visual odometry file to `.mat` format.
    - `readNovatel`: Convert Novatel pose file to `.mat` format.
    - `readIMU`: Convert IMU pose file to `.mat` format.

## 3. Interface Expansion

If you are using a different SLAM algorithm or a different INS device. You may define your own I/O interface accordingly. `pose2mat.m` can be used as a template.

## 4. Reference

1. [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)
2. [rosbag_to_csv](https://github.com/AtsushiSakai/rosbag_to_csv)
3. [COLMAP](https://github.com/colmap/colmap)
