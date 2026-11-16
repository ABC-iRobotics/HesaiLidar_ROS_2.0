# Local changes to the Hesai LiDAR ROS 2 driver (Óbuda Go2 setup)

This package runs the Hesai ROS 2 driver for the XT16 on the Go2's Jetson Orin, as part of the go2_kslam navigation stack.
It takes live UDP LiDAR input, decodes points on the GPU, and publishes `/lidar_points` (receive-time timestamps) for FAST-LIO.

## 1. Initial GPU / site configuration

- `config/config.yaml`
  - `use_gpu: true`: turns on GPU (CUDA) point decoding.
  - LiDAR IP set to `192.168.123.20`.
  - `use_timestamp_type: 1`: points are stamped with the time the packet was received, not the LiDAR's own clock.
  - `send_imu_ros: false`: IMU publishing disabled.
- `CMakeLists.txt`: CUDA detection enabled (`set(FIND_CUDA ON CACHE BOOL "Find CUDA")`).
- `src/manager/source_driver_ros2.hpp`: per-frame `printf` logging commented out.

## 2. CPU load reduction (2026-11-16)

### Why

The driver used about 19 % of one CPU core all the time. Measured per thread on the running stack:

| Thread | CPU | Cause |
|---|---|---|
| main | ~4 % | `while (...) sleep_for(100 µs)`: woke up about 5 500 times/s just to check for shutdown |
| UDP receive (SDK) | ~6.5 % | 5 000 packets/s on a non-blocking socket. Each time the socket runs dry it sleeps 1 ms, then calls `select`. The cost is mostly system calls. |
| SDK `Run()` | ~8 % | Decodes every packet (and copied each one into a list nothing uses), then once per frame: GPU XYZ step, CPU loop over every point, PointCloud2 build, DDS publish |

**The XT16 is in dual-return mode.** That means 500 packets per frame (5 000/s) and 64 000 points per frame (4 000 per ring), about 5 % of which are (0,0,0).
Each 1.66 MB frame goes to FAST-LIO (the only subscriber), about 17 MB/s at 10 Hz.

**GPU use.** The GPU is used, but only for the XYZ math once per frame. GPU load (GR3D in `tegrastats`) reads about 0 %, and the CPU still loops over every point afterwards.
GPU decoding is kept on: the CPU decoding path adds a per-packet hand-off to the parser threads, about 5 000 more thread wakeups per second.
The CUDA kernels were being built for `sm_61`. The SDK's CMake caches `CMAKE_CUDA_ARCHITECTURES=61`, and the earlier `sm_87` edit to `CUDA_NVCC_FLAGS` has no effect with CMake's native CUDA support.
As a result, the kernels were JIT-compiled from PTX on every start.

### What changed

| File | Change | Rationale |
|---|---|---|
| `node/hesai_ros_driver_node.cc` and `node/hesai_ros_driver_node.cu` | Main loop sleeps 100 ms instead of 100 µs; `sig_recv` is now `volatile std::sig_atomic_t` | The main thread only watches for SIGINT or end of PCAP, so this saves ~4 % CPU and changes no data. It doesn't wait on `g_cv`, because notifying a condition variable from a signal handler is not async-signal-safe. The CUDA build compiles the `.cu` copy of `main`, so **keep both files identical**. |
| `src/driver/HesaiLidar_SDK_2.0/driver/hesai_lidar_sdk.hpp` (`Run()`) | `udp_packet_frame.emplace_back(packet)` only when `pkt_cb_` is set | That list is used only for raw-packet publishing (`send_packet_ros`, which is off). Skipping it avoids a 1.5 KB copy per packet. |
| `src/manager/source_driver_ros2.hpp` (`ToRosMsg` for PointCloud2) | One raw-pointer pass at the fixed field offsets replaces six `PointCloud2Iterator`s; (0,0,0) points are dropped and `width`/`row_step`/`data` are shrunk to match | Cheaper per frame and smaller messages. The fields and their layout are unchanged (x, y, z, intensity float32; ring uint16; timestamp float64; 26 bytes per point). FAST-LIO's `blind` filter discarded the (0,0,0) points anyway. |
| `CMakeLists.txt` | `set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "" FORCE)` before CUDA is enabled; `-G;-g` removed from `CUDA_NVCC_FLAGS` | Native `sm_87` (Orin) code with no startup JIT. `-G` would turn off device-code optimisation if those flags were ever used. |
| `src/driver/HesaiLidar_SDK_2.0/tool/CMakeLists.txt` | `-G;-g` removed from `CUDA_NVCC_FLAGS` (arch was already `sm_87`) | Same reason as above. |

### Recommended LiDAR setting (not a code change)

Switch the XT16 to **single return** (web UI at `http://192.168.123.20` → Return Mode → Strongest or Last; the setting is stored on the LiDAR).
This halves the packets, the points, the DDS traffic (~8 MB/s), and FAST-LIO's preprocessing work.
FAST-LIO downsamples to a 0.2 m voxel grid anyway, so the second return adds little indoors. No FAST-LIO configuration change is needed.

### Expected result and verification

Estimate: from ~19 % to roughly 7–9 % of a core with single return plus the code changes. This has not been measured yet.

- Build: `colcon build --packages-select hesai_ros_driver`. Check that `build/hesai_ros_driver/CMakeFiles/hesai_ros_driver_node.dir/flags.make` shows `sm_87`.
- `top -H -p $(pgrep -f hesai_ros_driver_node)`: main-thread voluntary context switches should drop from ~5 500/s to ~10/s.
- `/lidar_points` still at 10 Hz. With single return, ~32k points per frame, no (0,0,0) points, same fields.
- FAST-LIO and slam_toolbox localization behave as before.

### Not done (possible next steps)

- Batching the SDK packet path (`recvmmsg`, popping several packets from the ring at once, one wakeup per batch). This gives the biggest remaining saving, but touches SDK internals and receive timestamps.
- `cudaDeviceScheduleBlockingSync`: the GPU wait is under 1 ms per frame at 10 Hz, so the gain is negligible.
