// IMPLEMENTATION OF UTILITIES INTERFACE
#include "utilities.h"
#include "mailbox.h"
#include <sys/time.h>

// TODO: Maybe this time is not paralel to ROS2 time
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
}