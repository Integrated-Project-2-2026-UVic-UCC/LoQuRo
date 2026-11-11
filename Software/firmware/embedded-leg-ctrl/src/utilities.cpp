// IMPLEMENTATION OF UTILITIES INTERFACE
#include "utilities.h"
#include "config.h"
#include "hardware.h"
#include "zenoh.h"
#include <sys/time.h>

// Only for sending
double getPreciseTime() // epoch time, synchronized with the PC clock published in robot/time
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

struct ScoutResult
{
    char locator[64];
};

// called for every zenoh node that answers the scouting. Keeps a locator reachable from our
// WiFi network (a PC also announces docker / IPv6 addresses), UDP if available
static void helloHandler(z_loaned_hello_t *hello, void *context)
{
    ScoutResult *result = (ScoutResult *)context;
    uint32_t mask = (uint32_t)WiFi.subnetMask();
    uint32_t network = (uint32_t)WiFi.localIP() & mask;

    z_owned_string_array_t locators;
    z_hello_locators(hello, &locators);
    const z_loaned_string_array_t *locs = z_string_array_loan(&locators);

    for (size_t i = 0; i < z_string_array_len(locs); i++)
    {
        const z_loaned_string_t *str = z_string_array_get(locs, i);
        char loc[64] = "";
        strncpy(loc, z_string_data(str), min(z_string_len(str), sizeof(loc) - 1));

        // "protocol/ip:port", only IPv4
        char protocol[4], ip_str[16];
        unsigned int port;
        IPAddress ip;
        if (sscanf(loc, "%3[a-z]/%15[0-9.]:%u", protocol, ip_str, &port) != 3 || !ip.fromString(ip_str))
            continue;

        bool udp = strcmp(protocol, "udp") == 0;
        if ((!udp && strcmp(protocol, "tcp") != 0) || ((uint32_t)ip & mask) != network)
            continue;

        if (result->locator[0] == '\0' || (udp && strncmp(result->locator, "udp", 3) != 0))
        {
            snprintf(result->locator, sizeof(result->locator), "%s/%s:%u", protocol, ip_str, port);
        }
    }
    z_string_array_drop(z_string_array_move(&locators));
}

// searchs the ROS2 bridge in the local network (zenoh multicast scouting), no IP needed
bool findRouter(char *locator, size_t locator_size)
{
    ScoutResult result = {""};

    z_owned_config_t config;
    z_config_default(&config);
    z_owned_closure_hello_t closure;
    z_closure_hello(&closure, helloHandler, NULL, &result);
    z_scout(z_config_move(&config), z_closure_hello_move(&closure), NULL); // blocks during the scouting timeout (1 s)

    if (result.locator[0] == '\0')
        return false;

    strlcpy(locator, result.locator, locator_size);
    return true;
}

void beginNetwork()
{
    setState(STATE_WAIT_WIFI);

    WiFiManager wm;
    wm.setConnectTimeout(10);
    wm.setConfigPortalTimeout(180);

    Serial.println("WIFI MANAGER: Searching for saved SSIDs");

    // tries the saved network, then opens the "LoQuRo" portal. If nobody configures it, tries again
    while (!wm.autoConnect("LoQuRo"))
        Serial.println("WIFI MANAGER: Not connected, retrying");

    WiFi.setSleep(false);

    setState(STATE_WAIT_ROS);

    char locator[64];

    Serial.println("ZENOH: Searching the ROS2 bridge in the network");
    while (!findRouter(locator, sizeof(locator)) || !initZenoh(locator))
        delay(1000);

    Serial.printf("SYSTEM: Zenoh connected succesfully in %s\n", locator);
}

// std_msgs/Bool and std_msgs/UInt8: CDR header + one byte
uint32_t serializeByte(uint8_t *buffer, uint8_t value)
{
    buffer[0] = 0x00;
    buffer[1] = 0x01; // Little Endian CDR
    buffer[2] = 0x00;
    buffer[3] = 0x00;
    buffer[4] = value;
    return 5;
}

// builtin_interfaces/Time: CDR header + int32 sec + uint32 nanosec
bool deserializeTime(const uint8_t *buffer, size_t len, double *time)
{
    if (len < 12)
        return false;
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, (uint8_t *)buffer + 4, len - 4);
    int32_t sec;
    uint32_t nanosec;
    ucdr_deserialize_int32_t(&ub, &sec);
    ucdr_deserialize_uint32_t(&ub, &nanosec);
    *time = (double)sec + (double)nanosec / 1e9;
    return true;
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

    { // frame id, by the moment we do not need it will be destroyed
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