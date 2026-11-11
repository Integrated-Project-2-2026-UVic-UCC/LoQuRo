#ifndef _TASKS_H_
#define _TASKS_H_
#include "zenoh.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void data_handler(z_loaned_sample_t *sample, void *arg);
void calibration_handler(z_loaned_sample_t *sample, void *arg);
void time_handler(z_loaned_sample_t *sample, void *arg);

// result of an imu calibration requested from ROS, published in imu/calibration_result
enum CalibrationResult : uint8_t
{
  CALIB_OK = 0,           // accel, gyro and magnetometer calibrated
  CALIB_NO_IMU = 1,       // accel/gyro not available, nothing done
  CALIB_NO_MAG = 2,       // accel and gyro calibrated, magnetometer not available
  CALIB_MAG_REJECTED = 3, // accel and gyro calibrated, the robot was not rotated enough (previous mag values kept)
  CALIB_BUSY = 4          // nothing done: the robot is not ready or it is receiving joint commands
};
extern void statusLedTask(void *pvParameters);
extern void watchdogTask(void *pvParameters);
extern void readIMUTask(void *pvParameters);
extern void sendStatusTask(void *pvParameters);
extern void writeServosTask(void *pvParameters);

extern const TickType_t whatchdog_delay;

extern bool imu_ok; // accel + gyro answering
extern bool mag_ok; // magnetometer giving data

#endif