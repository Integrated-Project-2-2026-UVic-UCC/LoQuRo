// IMPLEMENTATION OF ZENOH INTERFACE
#include "zenoh.h"
#include "config.h"
#include "tasks.h"

z_owned_session_t s;      // session object
z_owned_subscriber_t sub; // subsciber object

void initZenoh()
{
    // Initialize Zenoh Session and other parameters
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
    if (strcmp(LOCATOR, "") != 0)
    {
        if (strcmp(MODE, "client") == 0)
        {
            zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY,
                             LOCATOR);
        }
        else
        {
            zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_LISTEN_KEY,
                             LOCATOR);
        }
    }

    // Open Zenoh session
    Serial.print("Opening Zenoh Session...");
    if (z_open(&s, z_config_move(&config), NULL) < 0)
    {
        Serial.println("Unable to open session!");
        while (1)
        {
            ;
        }
    }
    Serial.println("OK");

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 ||
        zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0)
    {
        Serial.println("Unable to start read and lease tasks\n");
        z_session_drop(z_session_move(&s));
        while (1)
        {
            ;
        }
    }
    // Declare Zenoh subscriber
    Serial.print("Declaring Subscriber on ");
    Serial.print(KEYEXPR);
    Serial.println(" ...");
    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, data_handler, NULL, NULL);
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
    if (z_declare_subscriber(z_session_loan(&s), &sub, z_view_keyexpr_loan(&ke),
                             z_closure_sample_move(&callback), NULL) < 0)
    {
        Serial.println("Unable to declare subscriber.");
        while (1)
        {
            ;
        }
    }
    Serial.println("OK");
    Serial.print("Zenoh setup finished!\nIP: ");
}