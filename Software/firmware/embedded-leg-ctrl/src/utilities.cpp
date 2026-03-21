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

// TODO: function to deserialize float32Multiarray etc
bool deserializeJointStates(ucdrBuffer *ub)
{ // TODO: Calculo latencia
    // Header stamped
    uint32_t sec, nanosec;
    ucdr_deserialize_uint32_t(ub, &sec);
    ucdr_deserialize_uint32_t(ub, &nanosec);

    double timestamp =
        (double)sec +
        (double)nanosec / 1e9; // seconds + nanoseconds in seconds, epoch time

    xQueueOverwrite(Queues::time_stamp_queue,
                    &timestamp); // enviamos el timestamp a la tarea principal
                                 // para calcular la latencia

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
    uint32_t pos_count; // size of position array
    ucdr_deserialize_uint32_t(ub, &pos_count);

    if (pos_count < 12)
        return false; // we expect 12 positions, else the comunication failed or is
                      // not valid

    // CRÍTICO: Inicializar a 0.0 para evitar 'inf'
    double positions[12] = {0.0};

    // read all the postion array (we now we get doubles) (analized rawdata with
    // AI)
    if (!ucdr_deserialize_array_double(ub, positions, 12))
        return false;

    float leg_buf[4][3]; // buffer for the joint_states queue
    for (int leg = 0; leg < 4; leg++)
    {
        leg_buf[leg][0] = (float)positions[leg * 3 + 0];
        leg_buf[leg][1] = (float)positions[leg * 3 + 1];
        leg_buf[leg][2] = (float)positions[leg * 3 + 2];
    }
    xQueueOverwrite(Queues::joint_states_queue, leg_buf);
    return true;

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
    const double deg2rad = 3.14159265358979 / 180.0;
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