// CONFIG HEADER FILE
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h> // non-volatile memory
#include <Wire.h>

// Zenoh-specific parameters
#define MODE "client"

#define KEYEXPR "joint_commands"          // ros topic jointstates
#define IMU_KEYEXPR "imu/data_raw"        // ros topic imu
#define MAG_KEYEXPR "imu/mag"             // ros topic magnetic field
#define IMU_CALIB_KEYEXPR "imu/calibrate" // ros topic to request a new imu calibration (any message)
#define IMU_CALIB_RESULT_KEYEXPR "imu/calibration_result" // ros topic std_msgs/UInt8, see CalibrationResult
#define READY_KEYEXPR "robot/ready"       // ros topic std_msgs/Bool, true when joint commands are accepted (2 Hz)
#define TIME_KEYEXPR "robot/time"         // ros topic builtin_interfaces/Time, PC clock to timestamp the messages

// Valid band of (hfe - kfe) in rad, knee linkage (see Leg in hardware.h). Values of a LEFT leg: the right legs are
// mirrored (right = 1.6 - left), so they use the band with the sign changed (-DIFF_MAX, -DIFF_MIN).
// Measured on rf with test_leg_limits: -0.94 .. +0.91 (left convention), the MuJoCo model gives -0.95 .. +0.90.
// 0.10 rad of margin removed. Commands outside the band are moved to the nearest valid point
static constexpr float DIFF_MIN = -0.84f;
static constexpr float DIFF_MAX = 0.81f;

// joint position [haa, hfe, kfe] in rad that every leg takes when the communication with ROS is ready.
// The ROS nodes assume the robot starts here (initial_pose parameter of loquro_control): keep both equal
static constexpr float INIT_POSE[3] = {0.7f, 0.8f, 0.8f};

// brightness of the on-board RGB status led (0-255)
#define LED_BRIGHTNESS 40

#endif
