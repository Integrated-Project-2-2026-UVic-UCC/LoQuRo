// IMPLEMENTATION OF ZENOH INTERFACE (Inspired in oficial example codes of zenoh-pico)
#include "zenoh.h"
#include "config.h"
#include "tasks.h"

z_owned_session_t s;            // session object
z_owned_subscriber_t sub;       // subsciber object
z_owned_subscriber_t sub_calib; // subscriber imu calibration request
z_owned_publisher_t pub_imu;
z_owned_publisher_t pub_mag;
z_owned_publisher_t pub_ready;
z_owned_publisher_t pub_calib_result;
z_owned_subscriber_t sub_time;

// closes the session (and everything declared on it) when the setup fails, so it can be retried
static bool initFailed(const char *msg)
{
    Serial.println(msg);
    z_session_drop(z_session_move(&s));
    return false;
}

static bool declareSubscriber(z_owned_subscriber_t *subscriber, const char *keyexpr, z_closure_sample_callback_t handler)
{
    Serial.print("ZENOH: Declaring Subscriber on ");
    Serial.println(keyexpr);
    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, handler, NULL, NULL);
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, keyexpr);
    return z_declare_subscriber(z_session_loan(&s), subscriber, z_view_keyexpr_loan(&ke), z_closure_sample_move(&callback), NULL) >= 0;
}

static bool declarePublisher(z_owned_publisher_t *publisher, const char *keyexpr)
{
    Serial.print("ZENOH: Declaring Publisher on ");
    Serial.println(keyexpr);
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, keyexpr);
    return z_declare_publisher(z_session_loan(&s), publisher, z_view_keyexpr_loan(&ke), NULL) >= 0;
}

bool initZenoh(const char *locator_str)
{
    // Initialize Zenoh Session and other parameters
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, locator_str);

    // Open Zenoh session
    Serial.print("Opening Zenoh Session...");
    if (z_open(&s, z_config_move(&config), NULL) < 0)
    {
        Serial.println("Unable to open session!");
        return false;
    }
    Serial.println("OK");

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 ||
        zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0)
        return initFailed("Unable to start read and lease tasks");

    if (!declareSubscriber(&sub, KEYEXPR, data_handler))
        return initFailed("ZENOH: Unable to declare subscriber.");

    if (!declareSubscriber(&sub_calib, IMU_CALIB_KEYEXPR, calibration_handler))
        return initFailed("ZENOH: Unable to declare calibration subscriber.");

    if (!declareSubscriber(&sub_time, TIME_KEYEXPR, time_handler))
        return initFailed("ZENOH: Unable to declare time subscriber.");

    if (!declarePublisher(&pub_ready, READY_KEYEXPR))
        return initFailed("ZENOH: Error declaring ready publisher");

    if (!declarePublisher(&pub_calib_result, IMU_CALIB_RESULT_KEYEXPR))
        return initFailed("ZENOH: Error declaring calibration result publisher");

    if (!declarePublisher(&pub_imu, IMU_KEYEXPR))
        return initFailed("ZENOH: Error declaring IMU publisher");

    if (!declarePublisher(&pub_mag, MAG_KEYEXPR))
        return initFailed("ZENOH: Error declaring MAG publisher");

    Serial.println("SYSTEM: Zenoh setup finished!");
    return true;
}
