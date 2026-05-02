// IMPLEMENTATION OF ZENOH INTERFACE
#include "zenoh.h"
#include "config.h"
#include "tasks.h"

z_owned_session_t s;      // session object
z_owned_subscriber_t sub; // subsciber object
z_owned_publisher_t pub_imu;
z_owned_publisher_t pub_mag;
z_owned_publisher_t pub_encoder;

bool initZenoh(const char *locator_str)
{
    // Initialize Zenoh Session and other parameters
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
    if (strcmp(locator_str, "") != 0)
    {
        if (strcmp(MODE, "client") == 0)
        {
            zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY,
                             locator_str);
        }
        else
        {
            zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_LISTEN_KEY,
                             locator_str);
        }
    }

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
    {
        Serial.println("Unable to start read and lease tasks\n");
        z_session_drop(z_session_move(&s));
        return false;
    }
    // Declare Zenoh subscriber
    Serial.print("ZENOH: Declaring Subscriber on ");
    Serial.println(KEYEXPR);
    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, data_handler, NULL, NULL);
    z_view_keyexpr_t ke_sub;
    z_view_keyexpr_from_str_unchecked(&ke_sub, KEYEXPR);
    if (z_declare_subscriber(z_session_loan(&s), &sub, z_view_keyexpr_loan(&ke_sub), z_closure_sample_move(&callback), NULL) < 0)
    {
        Serial.println("ZENOH: Unable to declare subscriber.");
        return false;
    }

    // declare Zenoh IMu publisher
    Serial.print("ZENOH: Declaring Publisher on ");
    Serial.println(IMU_KEYEXPR);
    z_view_keyexpr_t ke_pub;
    z_view_keyexpr_from_str_unchecked(&ke_pub, IMU_KEYEXPR);
    if (z_declare_publisher(z_session_loan(&s), &pub_imu, z_view_keyexpr_loan(&ke_pub), NULL) < 0)
    {
        Serial.println("ZENOH: Error declaring IMU publisher");
        return false;
    }
    // declare zenoh mag publisher
    Serial.print("ZENOH: Declaring Publisher on ");
    Serial.println(MAG_KEYEXPR);
    z_view_keyexpr_t ke_pub_mag;
    z_view_keyexpr_from_str_unchecked(&ke_pub_mag, MAG_KEYEXPR);
    if (z_declare_publisher(z_session_loan(&s), &pub_mag, z_view_keyexpr_loan(&ke_pub_mag), NULL) < 0)
    {
        Serial.println("ZENOH: Error declaring MAG publisher");
        return false;
    }

    // declare zenoh mag publisher
    Serial.print("ZENOH: Declaring Publisher on ");
    Serial.println(ENCODER_KEYEXPR);
    z_view_keyexpr_t ke_pub_encoder;
    z_view_keyexpr_from_str_unchecked(&ke_pub_encoder, ENCODER_KEYEXPR);
    if (z_declare_publisher(z_session_loan(&s), &pub_encoder, z_view_keyexpr_loan(&ke_pub_encoder), NULL) < 0)
    {
        Serial.println("ZENOH: Error declaring ENCODER publisher");
        return false;
    }

    Serial.println("SYSTEM: Zenoh setup finished!");
    return true;
}
