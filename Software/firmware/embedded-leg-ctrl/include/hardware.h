// HARDWARE INTERFACE HEADER FILE
#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "PCA9685.h"
#include "IMUMPU9250.h"
#include "multiAS5600.h"

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

  void setLimit(float haa_limit, float hfe_limit, float kfe_limit)
  {
    haa.setLimit(haa_limit);
    hfe.setLimit(hfe_limit);
    kfe.setLimit(kfe_limit);
  }
  void write(float haa_rads, float hfe_rads, float kfe_rads)
  {
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

extern float joint_states[4][3];

// imu definition
extern IMUMPU9250 imu;

extern multiAS5600 joints;

#endif