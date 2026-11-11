#ifndef _ZENOH_H_
#define _ZENOH_H_
#include <zenoh-pico.h>

extern bool initZenoh(const char *locator_str);
extern z_owned_session_t s;             // session object
extern z_owned_subscriber_t sub;        // subsciber object
extern z_owned_subscriber_t sub_calib;  // subscriber imu calibration request
extern z_owned_publisher_t pub_imu;     // publisher imu
extern z_owned_publisher_t pub_mag;     // publisher magnetic_field
extern z_owned_publisher_t pub_ready;   // publisher robot ready
extern z_owned_publisher_t pub_calib_result; // publisher imu calibration result
extern z_owned_subscriber_t sub_time;   // subscriber PC clock

#endif