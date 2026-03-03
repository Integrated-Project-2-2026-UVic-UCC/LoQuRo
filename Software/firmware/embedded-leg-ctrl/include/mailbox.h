// MAILBOX INTERFACE HEADER FILE
#ifndef _MAILBOX_H_
#define _MAILBOX_H_

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace Queues
{
    extern QueueHandle_t joint_states_queue;
    extern QueueHandle_t time_stamp_queue;
    extern QueueHandle_t encoder_data_queue;
    extern QueueHandle_t imu_data_queue;
}

#endif