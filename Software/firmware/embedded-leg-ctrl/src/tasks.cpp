#include "tasks.h"
#include "config.h"
#include "hardware.h"
#include "mailbox.h"

const TickType_t whatchdog_delay = pdMS_TO_TICKS(500); // 2hz

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