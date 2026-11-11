// HARDWARE INTERFACE HEADER FILE
#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "PCA9685.h"
#include "IMUMPU9250.h"

struct Leg
{
  PCA9685Servo haa; // hip abductor adductor
  PCA9685Servo hfe; // hip flexor extensor
  PCA9685Servo kfe; // knee flexor extensor

  Leg(Adafruit_PWMServoDriver *driver, int haa_channel, int hfe_channel, int kfe_channel) // struct constructor
      : haa(driver, haa_channel),
        hfe(driver, hfe_channel),
        kfe(driver, kfe_channel)
  {
  }

  // The knee is driven through a linkage, so hfe and kfe are coupled: the mechanism only closes while
  // (hfe - kfe) stays inside a band, and it is singular at the edges. The limits live in this joint space
  float diff_min = -M_PI; // no limit by default
  float diff_max = M_PI;

  void setLimit(float haa_limit, float hfe_limit, float kfe_limit)
  {
    haa.setLimit(haa_limit);
    hfe.setLimit(hfe_limit);
    kfe.setLimit(kfe_limit);
  }
  void setCoupledLimit(float hfe_kfe_diff_min, float hfe_kfe_diff_max)
  {
    diff_min = hfe_kfe_diff_min;
    diff_max = hfe_kfe_diff_max;
  }
  void write(float haa_rads, float hfe_rads, float kfe_rads)
  {
    // command outside the band: move it to the nearest valid point (same correction in both joints)
    float diff = hfe_rads - kfe_rads;
    float excess = diff - constrain(diff, diff_min, diff_max);
    hfe_rads -= excess / 2.0f;
    kfe_rads += excess / 2.0f;

    haa.write(haa_rads);
    hfe.write(hfe_rads);
    kfe.write(kfe_rads);
  }
  void writeDegrees(float haa_degs, float hfe_degs, float kfe_degs)
  {
    haa.writeDegrees(haa_degs);
    hfe.writeDegrees(hfe_degs);
    kfe.writeDegrees(kfe_degs);
  }
};

namespace ServoParams
{
  // Channels of PCA board

  static constexpr uint8_t lf_haa_channel = 0;
  static constexpr uint8_t lf_hfe_channel = 1;
  static constexpr uint8_t lf_kfe_channel = 2;
  static constexpr uint8_t rf_haa_channel = 3;
  static constexpr uint8_t rf_hfe_channel = 4;
  static constexpr uint8_t rf_kfe_channel = 5;
  static constexpr uint8_t lh_haa_channel = 6;
  static constexpr uint8_t lh_hfe_channel = 7;
  static constexpr uint8_t lh_kfe_channel = 8;
  static constexpr uint8_t rh_haa_channel = 9;
  static constexpr uint8_t rh_hfe_channel = 10;
  static constexpr uint8_t rh_kfe_channel = 11;
}

extern Adafruit_PWMServoDriver pwm;

extern Leg leg_lf;
extern Leg leg_rf;
extern Leg leg_lh;
extern Leg leg_rh;

// imu definition
extern IMUMPU9250 imu;

// robot state, shown in the on-board RGB led
enum RobotState
{
  STATE_ERROR,        // red: hardware not operative
  STATE_WAIT_WIFI,    // blue blinking: waiting for WiFi (connecting or config portal open)
  STATE_WAIT_ROS,     // blue: WiFi ok, searching the ROS2 bridge
  STATE_CALIB_STILL,  // green blinking: calibrating gyro/accel, do not move
  STATE_CALIB_ROTATE, // green blinking fast: calibrating magnetometer, rotate the robot
  STATE_READY         // green: ready, joint commands are accepted. Yellow if a peripheral fails (see setDegraded)
};

void setState(RobotState state);
RobotState getState();
void setDegraded(bool degraded); // a peripheral (imu / magnetometer) fails, the robot can still work: yellow led
bool isDegraded();

#endif