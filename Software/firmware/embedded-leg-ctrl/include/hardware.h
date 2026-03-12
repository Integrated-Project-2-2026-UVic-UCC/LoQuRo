// HARDWARE INTERFACE HEADER FILE
#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "ServoCtrl.h"

struct Leg {
  ServoCtrl haa;  // hip abductor adductor
  ServoCtrl hfe;  // hip flexor extensor
  ServoCtrl kfe;  // knee flexor extensor

  Leg(int haa_pin, int hfe_pin, int kfe_pin, int haa_channel, int hfe_channel,
      int kfe_channel, int resolution)
      : haa(haa_pin, haa_channel, resolution),
        hfe(hfe_pin, hfe_channel, resolution),
        kfe(kfe_pin, kfe_channel, resolution) {}
  void setLimit(float haa_limit, float hfe_limit, float kfe_limit) {
    haa.setLimit(haa_limit);
    hfe.setLimit(hfe_limit);
    kfe.setLimit(kfe_limit);
  }
  void write(float haa_rads, float hfe_rads, float kfe_rads) {
    haa.write(haa_rads);
    hfe.write(hfe_rads);
    kfe.write(kfe_rads);
  }
  void writeDegrees(float haa_degs, float hfe_degs, float kfe_degs) {
    haa.writeDegrees(haa_degs);
    hfe.writeDegrees(hfe_degs);
    kfe.writeDegrees(kfe_degs);
  }
};

namespace ServoParams {
static constexpr int lf_hha_pin = 16;
static constexpr int lf_hfe_pin = 17;
static constexpr int lf_kfe_pin = 18;
static constexpr int rf_hha_pin = 19;
static constexpr int rf_hfe_pin = 21;
static constexpr int rf_kfe_pin = 22;
static constexpr int lh_hha_pin = 23;
static constexpr int lh_hfe_pin = 25;
static constexpr int lh_kfe_pin = 26;
static constexpr int rh_hha_pin = 27;
static constexpr int rh_hfe_pin = 32;
static constexpr int rh_kfe_pin = 33;

static constexpr int lf_haa_channel = 0;   // Canal LEDC (0-15)
static constexpr int lf_hfe_channel = 1;   // Canal LEDC (0-15)
static constexpr int lf_kfe_channel = 2;   // Canal LEDC (0-15)
static constexpr int rf_haa_channel = 3;   // Canal LEDC (0-15)
static constexpr int rf_hfe_channel = 4;   // Canal LEDC (0-15)
static constexpr int rf_kfe_channel = 5;   // Canal LEDC (0-15)
static constexpr int lh_haa_channel = 6;   // Canal LEDC (0-15)
static constexpr int lh_hfe_channel = 7;   // Canal LEDC (0-15)
static constexpr int lh_kfe_channel = 8;   // Canal LEDC (0-15)
static constexpr int rh_haa_channel = 9;   // Canal LEDC (0-15)
static constexpr int rh_hfe_channel = 10;  // Canal LEDC (0-15)
static constexpr int rh_kfe_channel = 11;  // Canal LEDC (0-15)

constexpr int resolution = 16;  // Resolución (1-16 bits)
}  // namespace ServoParams

extern Leg leg_lf;
extern Leg leg_rf;
extern Leg leg_lh;
extern Leg leg_rh;

extern float joint_states[4][3];

#endif