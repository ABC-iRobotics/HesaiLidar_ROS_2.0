# Hesai LiDAR GPU Config Summary

This package was configured to run the Hesai ROS 2 driver with GPU/CUDA support.

## Edited Files

- `config/config.yaml`
  - Enabled GPU parsing with `use_gpu: true`.
  - Set LiDAR IP to `192.168.123.20`.
  - Set `use_timestamp_type: 1` to use receive timestamps.
  - Disabled IMU publishing with `send_imu_ros: false`.

- `CMakeLists.txt`
  - Enabled CUDA detection with `set(FIND_CUDA ON CACHE BOOL "Find CUDA")`.
  - Changed CUDA architecture from `sm_61` to `sm_87` for Jetson Orin / Ampere GPU support.

- `src/driver/HesaiLidar_SDK_2.0/tool/CMakeLists.txt`
  - Changed CUDA architecture from `sm_61` to `sm_87` for SDK tool builds.

- `src/manager/source_driver_ros2.hpp`
  - Commented out per-frame `printf` logging to reduce terminal spam and runtime overhead.

## Result

The driver is set up for live UDP LiDAR input, GPU point cloud parsing, ROS 2 point cloud publishing, receive-time timestamps, and reduced console output.
