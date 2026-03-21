// IMPLEMENTAION OF THE HARDWARE INTERFACE
#include <hardware.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1); // PCA instance

// struct instances
Leg leg_lf(&pwm, ServoParams::lf_haa_channel, ServoParams::lf_hfe_channel, ServoParams::lf_kfe_channel);
Leg leg_rf(&pwm, ServoParams::rf_haa_channel, ServoParams::rf_hfe_channel, ServoParams::rf_kfe_channel);
Leg leg_lh(&pwm, ServoParams::lh_haa_channel, ServoParams::lh_hfe_channel, ServoParams::lh_kfe_channel);
Leg leg_rh(&pwm, ServoParams::rh_haa_channel, ServoParams::rh_hfe_channel, ServoParams::rh_kfe_channel);

float joint_states[4][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

// imu implementation
IMUMPU9250 imu(0x68, 0x0C);