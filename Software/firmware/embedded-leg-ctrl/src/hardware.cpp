// IMPLEMENTAION OF THE HARDWARE INTERFACE
#include <hardware.h>

Leg leg_lf(ServoParams::lf_hha_pin, ServoParams::lf_hfe_pin, ServoParams::lf_kfe_pin, ServoParams::lf_haa_channel, ServoParams::lf_hfe_channel, ServoParams::lf_kfe_channel, ServoParams::resolution);
Leg leg_rf(ServoParams::rf_hha_pin, ServoParams::rf_hfe_pin, ServoParams::rf_kfe_pin, ServoParams::rf_haa_channel, ServoParams::rf_hfe_channel, ServoParams::rf_kfe_channel, ServoParams::resolution);
Leg leg_lh(ServoParams::lh_hha_pin, ServoParams::lh_hfe_pin, ServoParams::lh_kfe_pin, ServoParams::lh_haa_channel, ServoParams::lh_hfe_channel, ServoParams::lh_kfe_channel, ServoParams::resolution);
Leg leg_rh(ServoParams::rh_hha_pin, ServoParams::rh_hfe_pin, ServoParams::rh_kfe_pin, ServoParams::rh_haa_channel, ServoParams::rh_hfe_channel, ServoParams::rh_kfe_channel, ServoParams::resolution);

float joint_states[4][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}; // LF, RF, LH, RH - HAA, HFE, KFE. It will be changeing, cannot be static