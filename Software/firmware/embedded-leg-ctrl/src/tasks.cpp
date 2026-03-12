// IMPLEMENTATION OF TASKS INTERFACE
#include "tasks.h"
#include "config.h"
#include "hardware.h"
#include "mailbox.h"
#include "utilities.h"

const TickType_t whatchdog_delay = pdMS_TO_TICKS(500); // 2hz

// leg motors
uint8_t raw_buffer[512]; // buffer of 512 bytes to recieve raw data

// zenoh callback to receive data
void data_handler(z_loaned_sample_t *sample, void *arg)
{
    const z_loaned_bytes_t *payload = z_sample_payload(sample);
    z_bytes_reader_t reader = z_bytes_get_reader(payload);
    size_t len = z_bytes_reader_remaining(&reader);

    if (len < 32 || len > sizeof(raw_buffer))
    {
        Serial.println(len); // the len is dynamic so if the names changes we
                             // cannot expect a fixed size
        return;              // only accept messages larger than 32 bytes (else there is no
                             // valid data)
    }
    z_bytes_reader_read(&reader, raw_buffer, len);

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, raw_buffer + 4,
                     len - 4); // pass first 4 bytes, header of message

    if (!deserializeJointStates(&ub))
        return;

    //---DEBUGGING---
    // for (int i = 0; i < len / sizeof(uint8_t); ++i)
    // {
    //   Serial.print(((uint8_t *)raw_buffer)[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();
}

void watchdogTask(void *pvParameters)
{
    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            digitalWrite(LED_PIN, LOW);
            WiFi.reconnect();                // DONE: It do not reconnects, maybe we need to do WiFi.begin() again?
            vTaskDelay(pdMS_TO_TICKS(3000)); // greater delay to give time to reconnect
        }
        else
        {
            if (digitalRead(LED_PIN) == LOW) // if wifi was disconnected, turn on LED when reconnected
            {
                digitalWrite(LED_PIN, HIGH);
            }
            vTaskDelay(whatchdog_delay); // delay to not overload CPU
        }
    }
}

void writeServosTask(void *pvParameters)
{
    for (;;)
    {
        if (xQueueReceive(Queues::joint_states_queue, &joint_states, portMAX_DELAY) == pdTRUE) // if there is data in the buffer
        {
            // MOVE SERVOS
            leg_lf.write(joint_states[0][0], joint_states[0][1], joint_states[0][2]);
            leg_rf.write(joint_states[1][0], joint_states[1][1], joint_states[1][2]);
            leg_lh.write(joint_states[2][0], joint_states[2][1], joint_states[2][2]);
            leg_rh.write(joint_states[3][0], joint_states[3][1], joint_states[3][2]);
        }
    }
}

void readEncodersTask(void *pvParameters)
{
    for (;;)
    {
        // TODO: read encoders and pin to core
        vTaskDelay(pdMS_TO_TICKS(10)); // 100hz
    }
}

void readIMUTask(void *pvParameters)
{
    for (;;)
    {
        // TODO: read IMU and pin to core
        vTaskDelay(pdMS_TO_TICKS(10)); // 100hz
    }
}

void sendStatusTask(void *pvParameters)
{
    for (;;)
    {
        // TODO: Send status (encoder data, imu data) to ROS2 through zenoh-pico and pin to core
        vTaskDelay(pdMS_TO_TICKS(10)); // ajustar frecuencia de envio de status
    }
}