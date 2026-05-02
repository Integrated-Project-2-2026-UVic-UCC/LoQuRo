// IMPLEMENTATION OF UTILITIES INTERFACE
#include "utilities.h"
#include "config.h"
#include "mailbox.h"
#include "zenoh.h"
#include <sys/time.h>

bool shouldSaveConfig = false;

// Only for sending
double getPreciseTime() // standard function to get epoch time via NTP
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

void saveConfigCallback()
{
    Serial.println("WIFI MANAGER: Web portal changes detected, flag activated");
    shouldSaveConfig = true;
}

void beginNetwork()
{
    Preferences preferences;
    char pc_ip[40] = ""; // will save IP
    preferences.begin("robot", false);
    String saved_ip = preferences.getString("zenoh_ip", "");
    strncpy(pc_ip, saved_ip.c_str(), sizeof(pc_ip));

    WiFiManager wm;
    wm.setSaveConfigCallback(saveConfigCallback);
    wm.setConnectTimeout(10);
    wm.setConfigPortalTimeout(180);

    WiFiManagerParameter custom_zenoh_ip("zenoh", "PC IP (ROS2)", pc_ip, 40);
    wm.addParameter(&custom_zenoh_ip);

    Serial.println("WIFI MANAGER: Searching for saved SSIDs");

    if (!wm.autoConnect("Quad-bot"))
    {
        Serial.println("WIFI MANAGER: Timeout, resetting device");
        delay(3000);
        ESP.restart();
    }

    // if user saved data correctly from web
    if (shouldSaveConfig)
    {
        String new_ip = custom_zenoh_ip.getValue();
        if (new_ip.length() >= 7)
        {
            strncpy(pc_ip, new_ip.c_str(), sizeof(pc_ip));
            preferences.putString("zenoh_ip", pc_ip);
            Serial.println("WIFI MANAGER: New network saved");
        }
        shouldSaveConfig = false; // not necessary to save config
    }

    WiFi.setSleep(false);

    char full_locator[60] = "";
    bool zenoh_connected = false;

    if (strlen(pc_ip) >= 7)
    {
        sprintf(full_locator, "tcp/%s:7447", pc_ip);
        Serial.printf("ZENOH: Trying to connect Zenoh in: %s\n", full_locator);
        zenoh_connected = initZenoh(full_locator);
    }

    if (!zenoh_connected)
    {
        Serial.println("ZENOH: Zenoh initialization failed! Opening web portal");
        digitalWrite(LED_PIN, LOW);

        if (wm.startConfigPortal("Quad-bot"))
        {
            if (shouldSaveConfig)
            {
                String forced_ip = custom_zenoh_ip.getValue();
                if (forced_ip.length() >= 7)
                {
                    strncpy(pc_ip, forced_ip.c_str(), sizeof(pc_ip));
                    preferences.putString("zenoh_ip", pc_ip);
                    Serial.printf("WIFI MANAGER: New IP saved correctly!: %s\n", pc_ip);
                }
                else
                {
                    Serial.println("WIFI MANAGER: IP not valid! Will be ignored");
                }
            }
            Serial.println("SYSTEM: Resetting device to apply changes");
            delay(2000);
            ESP.restart();
        }
        else
        {
            Serial.println("WIFI MANAGER: Timeout reached without configuration, resetting device");
            delay(2000);
            ESP.restart();
        }
    }
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    // Esperar sincronización
    Serial.print("Syncing NTP");
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo))
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println(" OK");

    Serial.println("SYSTEM: Zenoh connected succesfully!");
}

// FEATURE: function to deserialize float32Multiarray etc
bool deserializeJointStates(ucdrBuffer *ub, float target_joints[4][3], double *timestamp)
{
    // Header stamped
    uint32_t sec, nanosec;
    ucdr_deserialize_uint32_t(ub, &sec);
    ucdr_deserialize_uint32_t(ub, &nanosec);

    *timestamp =
        (double)sec +
        (double)nanosec / 1e9; // seconds + nanoseconds in seconds, epoch time

    { // Frame id, by the moment we do not need it will be destroyed
        char frame_id[32];
        ucdr_deserialize_string(ub, frame_id, sizeof(frame_id));
    }

    { // all created vbles will be destroyed after this block
        uint32_t names_count;
        ucdr_deserialize_uint32_t(ub,
                                  &names_count); // number of names in the buffer
        char name_buf[32];
        for (uint32_t i = 0; i < names_count;
             i++) // read number of names and discard
            ucdr_deserialize_string(ub, name_buf, sizeof(name_buf));
    }
    uint32_t pos_count;
    ucdr_deserialize_uint32_t(ub, &pos_count);
    if (pos_count < 12)
        return false;

    ucdr_align_to(ub, 8);

    double positions[12] = {0.0};
    if (!ucdr_deserialize_array_double(ub, positions, 12))
        return false;

    for (int leg = 0; leg < 4; leg++)
    {
        target_joints[leg][0] = (float)positions[leg * 3 + 0];
        target_joints[leg][1] = (float)positions[leg * 3 + 1];
        target_joints[leg][2] = (float)positions[leg * 3 + 2];
    }

    return true;
}

uint32_t serializeJointStates(uint8_t *buffer, uint32_t size, const float target_joints[4][3], double timestamp)
{
    // CDR header
    buffer[0] = 0x00;
    buffer[1] = 0x01; // Little Endian CDR
    buffer[2] = 0x00;
    buffer[3] = 0x00;

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer + 4, size - 4);

    // Header stamp
    uint32_t sec = (uint32_t)timestamp;
    uint32_t nanosec = (uint32_t)((timestamp - sec) * 1e9);
    ucdr_serialize_uint32_t(&ub, sec);
    ucdr_serialize_uint32_t(&ub, nanosec);

    // Frame id (empty string as it's not used)
    ucdr_serialize_string(&ub, "");

    // Names count (0 as names are not serialized)
    uint32_t names_count = 0;
    ucdr_serialize_uint32_t(&ub, names_count);

    // Positions count (always 12 for 4 legs with 3 joints each)
    uint32_t pos_count = 12;
    ucdr_serialize_uint32_t(&ub, pos_count);

    // Align buffer to 8 bytes para el array de doubles
    ucdr_align_to(&ub, 8);

    // Serialize joint positions
    double positions[12];
    for (int leg = 0; leg < 4; leg++)
    {
        positions[leg * 3 + 0] = (double)target_joints[leg][0];
        positions[leg * 3 + 1] = (double)target_joints[leg][1];
        positions[leg * 3 + 2] = (double)target_joints[leg][2];
    }
    ucdr_serialize_array_double(&ub, positions, 12);

    uint32_t empty_count = 0;
    ucdr_serialize_uint32_t(&ub, empty_count); // velocity count = 0
    ucdr_serialize_uint32_t(&ub, empty_count); // effort count = 0

    return 4 + ucdr_buffer_length(&ub);
}

uint32_t serializeImu(uint8_t *buffer, uint32_t size, const IMUdata &data, double timestamp)
{
    // it gets auto headered, out of buffer
    buffer[0] = 0x00;
    buffer[1] = 0x01; // Little Endian CDR
    buffer[2] = 0x00;
    buffer[3] = 0x00;

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer + 4, size - 4);

    // Header stamp
    uint32_t sec = (uint32_t)timestamp;
    uint32_t nanosec = (uint32_t)((timestamp - sec) * 1e9);
    ucdr_serialize_uint32_t(&ub, sec);
    ucdr_serialize_uint32_t(&ub, nanosec);

    // no frame id
    ucdr_serialize_string(&ub, "imu_link");

    // orientation in cuaternions, 0 by default
    ucdr_serialize_double(&ub, 0.0);
    ucdr_serialize_double(&ub, 0.0);
    ucdr_serialize_double(&ub, 0.0);
    ucdr_serialize_double(&ub, 1.0); // initial angle

    // Orientation covariance: -1 en [0,0] = "no tengo orientación", Madgwick la calcula
    double cov_orient[9] = {-1.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0};
    ucdr_serialize_array_double(&ub, cov_orient, 9);

    // Angular velocity (rad/s)
    const double deg2rad = M_PI / 180.0;
    ucdr_serialize_double(&ub, (double)data.gyro.x * deg2rad);
    ucdr_serialize_double(&ub, (double)data.gyro.y * deg2rad);
    ucdr_serialize_double(&ub, (double)data.gyro.z * deg2rad);

    // Gyro covariance: MPU9250 datasheet ~0.05 dps rms at 500dps scale -> (0.05 * deg2rad)^2
    const double gyro_var = 7.6e-7; // (rad/s)^2
    double cov_gyro[9] = {gyro_var, 0.0, 0.0,
                          0.0, gyro_var, 0.0,
                          0.0, 0.0, gyro_var};
    ucdr_serialize_array_double(&ub, cov_gyro, 9);

    // Linear acceleration (m/s²)
    const double g_to_ms2 = 9.80665;
    ucdr_serialize_double(&ub, (double)data.accel.x * g_to_ms2);
    ucdr_serialize_double(&ub, (double)data.accel.y * g_to_ms2);
    ucdr_serialize_double(&ub, (double)data.accel.z * g_to_ms2);

    // Accel covariance: MPU9250 datasheet ~8mg rms at +-8g scale -> (8e-3 * 9.80665)^2
    const double accel_var = 6.2e-3; // (m/s^2)^2
    double cov_accel[9] = {accel_var, 0.0, 0.0,
                           0.0, accel_var, 0.0,
                           0.0, 0.0, accel_var};
    ucdr_serialize_array_double(&ub, cov_accel, 9);

    // +4 cause we removed the header
    return 4 + ucdr_buffer_length(&ub);
}

uint32_t serializeMag(uint8_t *buffer, uint32_t size, const IMUdata &data, double timestamp)
{
    // CDR header
    buffer[0] = 0x00;
    buffer[1] = 0x01; // Little Endian CDR
    buffer[2] = 0x00;
    buffer[3] = 0x00;

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer + 4, size - 4);

    // Header stamp
    uint32_t sec = (uint32_t)timestamp;
    uint32_t nanosec = (uint32_t)((timestamp - sec) * 1e9);
    ucdr_serialize_uint32_t(&ub, sec);
    ucdr_serialize_uint32_t(&ub, nanosec);

    // no frame id needed
    ucdr_serialize_string(&ub, "imu_link");

    // magnetic_field (geometry_msgs/Vector3) in teslas, sensor gives uTeslas
    const double uT_to_T = 1e-6;
    ucdr_serialize_double(&ub, (double)data.mag.x * uT_to_T);
    ucdr_serialize_double(&ub, (double)data.mag.y * uT_to_T);
    ucdr_serialize_double(&ub, (double)data.mag.z * uT_to_T);

    // no covariance
    double cov_null[9] = {0.0};
    ucdr_serialize_array_double(&ub, cov_null, 9);

    return 4 + ucdr_buffer_length(&ub);
}