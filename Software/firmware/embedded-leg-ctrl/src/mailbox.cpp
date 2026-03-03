// IMPLEMENTAION OF THE MAILBOX
#include "mailbox.h"

QueueHandle_t Queues::joint_states_queue;
QueueHandle_t Queues::time_stamp_queue;
QueueHandle_t Queues::encoder_data_queue;
QueueHandle_t Queues::imu_data_queue;