# LoQuRo changes

Copy of `imu_filter_madgwick` from [imu_tools](https://github.com/CCNYRoboticsLab/imu_tools), branch `humble`,
commit `1711f25` (version 2.1.6). License GPL-3.0-or-later, same as LoQuRo.

Only change: with `use_mag: true`, the original filter needs the IMU and magnetometer messages paired and stops
publishing if the magnetometer stops. Now, when there are no magnetometer messages for `mag_timeout` seconds
(new parameter, default 0.5), it keeps filtering with accelerometer + gyroscope, and goes back to using the
magnetometer when it is received again. Code marked with `LoQuRo:` in `src/imu_filter_ros.cpp` and
`include/imu_filter_madgwick/imu_filter_ros.h`.

The upstream tests and sample data were removed.
