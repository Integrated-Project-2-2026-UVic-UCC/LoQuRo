// IMPLEMENTATION OF TASKS INTERFACE
#include "tasks.h"
#include "config.h"
#include "hardware.h"
#include "mailbox.h"
#include "utilities.h"

const TickType_t whatchdog_delay = pdMS_TO_TICKS(5000); // 0.2hz

// zenoh callback to receive data
void data_handler(z_loaned_sample_t *sample, void *arg)
{
    uint8_t raw_buffer[512]; // buffer of 512 bytes to recieve raw data
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

    float new_joints[4][3];
    double new_timestamp;

    if (deserializeJointStates(&ub, new_joints, &new_timestamp))
    {
        // if the message got deserialized, send to each queue
        xQueueOverwrite(Queues::joint_states_queue, &new_joints);
        // timestamp mirar dequitar
        xQueueOverwrite(Queues::time_stamp_queue, &new_timestamp);
    }

    // //---DEBUGGING---
    // for (int i = 0; i < len / sizeof(uint8_t); ++i)
    // {
    //     Serial.print(((uint8_t *)raw_buffer)[i], HEX);
    //     Serial.print(" ");
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
    float new_joints[4][3];
    leg_lf.write(0.0, 0.0, 0.0); // for debug, to not move the robot
    leg_rf.write(0.0, 0.0, 0.0);
    leg_lh.write(0.0, 0.0, 0.0);
    leg_rh.write(0.0, 0.0, 0.0);
    for (;;)
    {
        if (xQueueReceive(Queues::joint_states_queue, &new_joints, portMAX_DELAY) == pdTRUE)
        {
            memcpy(joint_states, new_joints, sizeof(joint_states)); // only for debug, race conditiin doesnt matter
            leg_lf.write(new_joints[0][0], new_joints[0][1], new_joints[0][2]);
            leg_rf.write(new_joints[1][0], new_joints[1][1], new_joints[1][2]);
            leg_lh.write(new_joints[2][0], new_joints[2][1], new_joints[2][2]);
            leg_rh.write(new_joints[3][0], new_joints[3][1], new_joints[3][2]);
        }
        // frecuencia de recepcion hasta el maxportdelay
    }
}

void readEncodersTask(void *pvParameters)
{
    float angles[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (;;)
    {
        for (uint8_t port = 0; port < 8; port++)
        {
            // Select mux port
            Wire.beginTransmission(0x70);
            Wire.write(1 << port);
            Wire.endTransmission();

            // Check if encoder (0x36) is present on this port
            Wire.beginTransmission(0x36);
            if (Wire.endTransmission() == 0)
            {
                // Encoder found, read angle
                Wire.beginTransmission(0x36);
                Wire.write(0x0E); // angle register
                Wire.endTransmission(false);

                Wire.requestFrom((uint8_t)0x36, (uint8_t)2);

                if (Wire.available() >= 2)
                {
                    uint8_t hi = Wire.read();
                    uint8_t lo = Wire.read();

                    uint16_t raw_angle = ((uint16_t)(hi & 0x0F) << 8) | lo;
                    angles[port] = static_cast<float>(raw_angle) * 360.0f / 4096.0f;
                }
            }
            else
            {
                angles[port] = NAN;
            }
        }

        xQueueOverwrite(Queues::encoder_data_queue, &angles);
        vTaskDelay(pdMS_TO_TICKS(100)); // 10hz
    }
}

void readIMUTask(void *pvParameters)
{
    IMUdata current_data;
    for (;;)
    {
        current_data.accel = imu.getAccel();
        current_data.gyro = imu.getGyro();
        current_data.mag = imu.getMag();

        xQueueOverwrite(Queues::imu_data_queue, &current_data);
        vTaskDelay(pdMS_TO_TICKS(10)); // 100hz
    }
}

void sendStatusTask(void *pvParameters)
{
    // TODO: Add encoders
    IMUdata received_imu;
    uint8_t imu_cdr_buffer[512];
    uint8_t mag_cdr_buffer[256];
    uint8_t encoder_cdr_buffer[512];
    float received_encoder_angles[12];
    float encoder_joints[4][3]; // reshape flat encoder angles to [leg][joint]
    for (;;)
    {
        // TODO: Send encoder data to ROS2 through zenoh-pico and pin to core
        if (xQueueReceive(Queues::imu_data_queue, &received_imu, portMAX_DELAY) == pdTRUE) // if there is data in the buffer
        {
            // get current time
            double now = getPreciseTime();

            // serialize to IMU msg
            uint32_t msg_len = serializeImu(imu_cdr_buffer, sizeof(imu_cdr_buffer), received_imu, now);

            // zenoh payload
            z_owned_bytes_t payload;
            z_bytes_copy_from_buf(&payload, imu_cdr_buffer, msg_len);

            // publish in imu keyexpr
            static uint32_t pub_ok = 0, pub_fail = 0;
            if (z_publisher_put(z_publisher_loan(&pub_imu), z_bytes_move(&payload), NULL) < 0)
            {
                pub_fail++;
                if (pub_fail % 10 == 0)
                    Serial.printf("ZENOH: %lu ok / %lu fail\n", pub_ok, pub_fail);
            }
            else
            {
                pub_ok++;
            }

            // same for magneitc field msg
            uint32_t mag_len = serializeMag(mag_cdr_buffer, sizeof(mag_cdr_buffer), received_imu, now);
            z_owned_bytes_t mag_payload; // TODO: Add encoders
            z_bytes_copy_from_buf(&mag_payload, mag_cdr_buffer, mag_len);
            if (z_publisher_put(z_publisher_loan(&pub_mag), z_bytes_move(&mag_payload), NULL) < 0)
            {
                Serial.println("ZENOH: Error publishing MAG data");
            }

            //---DEBUGGING---
            // Serial.p -+rint("ACCELEROMETER: ");
            // (received_imu.accel.x < 0.01 && received_imu.accel.x > -0.01) ? Serial.print(abs(received_imu.accel.x)) : Serial.print(received_imu.accel.x); // to do not print negative 0.0, otherwise is difficult to see
            // Serial.print(", ");
            // (received_imu.accel.y < 0.01 && received_imu.accel.y > -0.01) ? Serial.print(abs(received_imu.accel.y)) : Serial.print(received_imu.accel.y);
            // Serial.print(", ");
            // (received_imu.accel.z < 0.01 && received_imu.accel.z > -0.01) ? Serial.print(abs(received_imu.accel.z)) : Serial.print(received_imu.accel.z);
            // Serial.print("  GYROSCOPE: ");
            // Serial.print(received_imu.gyro.x);
            // Serial.print(", ");
            // Serial.print(received_imu.gyro.y);
            // Serial.print(", ");
            // Serial.println(received_imu.gyro.z);
            // Serial.print("  MAGNETOMETER: ");
            // Serial.print(received_imu.mag.x);
            // Serial.print(", ");
            // Serial.print(received_imu.mag.y);
            // Serial.print(", ");
            // Serial.println(received_imu.mag.z);
        }
        // publish encoder data if available (non-blocking peek)
        if (xQueueReceive(Queues::encoder_data_queue, &received_encoder_angles, 0) == pdTRUE) // no port delay, temes de sincronitzacio
        {
            // reshape flat array [12] to [4][3] matching leg/joint layout
            for (int leg = 0; leg < 4; leg++)
            {
                encoder_joints[leg][0] = received_encoder_angles[leg * 3 + 0];
                encoder_joints[leg][1] = received_encoder_angles[leg * 3 + 1];
                encoder_joints[leg][2] = received_encoder_angles[leg * 3 + 2];
            }

            double now = getPreciseTime();

            uint32_t enc_len = serializeJointStates(encoder_cdr_buffer, sizeof(encoder_cdr_buffer), encoder_joints, now);
            z_owned_bytes_t enc_payload;
            z_bytes_copy_from_buf(&enc_payload, encoder_cdr_buffer, enc_len);
            if (z_publisher_put(z_publisher_loan(&pub_encoder), z_bytes_move(&enc_payload), NULL) < 0)
            {
                Serial.println("ZENOH: Error publishing encoder data");
            }
        }
    }
}