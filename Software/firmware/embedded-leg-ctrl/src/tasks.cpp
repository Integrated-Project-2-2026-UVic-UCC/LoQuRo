// IMPLEMENTATION OF TASKS INTERFACE
#include "tasks.h"
#include "config.h"
#include "hardware.h"
#include "mailbox.h"
#include "utilities.h"
#include <sys/time.h>

const TickType_t whatchdog_delay = pdMS_TO_TICKS(5000); // 0.2hz

bool imu_ok = false;
bool mag_ok = false;

static constexpr uint32_t mag_timeout_ms = 200;    // without valid mag samples for this time -> magnetometer lost
static constexpr uint8_t imu_max_fails = 10;       // consecutive failed accel/gyro reads (100 ms) -> imu lost
static constexpr uint32_t sensor_retry_ms = 2000;  // period to try to recover a lost sensor
static constexpr uint32_t calib_idle_ms = 2000;    // time without joint commands needed to accept a calibration

static volatile uint32_t last_command_ms = 0; // last joint command received
static volatile bool command_received = false;
static volatile bool clock_synced = false;    // robot/time received: the timestamps are valid

// zenoh callback to receive data
void data_handler(z_loaned_sample_t *sample, void *arg)
{
    uint8_t raw_buffer[512]; // buffer of 512 bytes to recieve raw data
    const z_loaned_bytes_t *payload = z_sample_payload(sample);
    z_bytes_reader_t reader = z_bytes_get_reader(payload);
    size_t len = z_bytes_reader_remaining(&reader);

    // commands are only accepted when the robot is ready (not while booting or calibrating)
    if (getState() != STATE_READY)
        return;

    // the len is dynamic so if the names changes we cannot expect a fixed size.
    // Only accept messages larger than 32 bytes (else there is no valid data)
    if (len < 32 || len > sizeof(raw_buffer))
        return;
    z_bytes_reader_read(&reader, raw_buffer, len);

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, raw_buffer + 4,
                     len - 4); // pass first 4 bytes, header of message

    float new_joints[4][3];
    double new_timestamp;

    if (deserializeJointStates(&ub, new_joints, &new_timestamp))
    {
        // if the message got deserialized, send to the queue
        xQueueOverwrite(Queues::joint_states_queue, &new_joints);
        last_command_ms = millis();
        command_received = true;
    }
}

// zenoh callback: any message in the calibration topic requests a new imu calibration
static volatile bool imu_calibration_requested = false;

void calibration_handler(z_loaned_sample_t *sample, void *arg)
{
    imu_calibration_requested = true;
}

// zenoh callback: PC clock (1 Hz). The first message sets the clock, then it is slewed smoothly so the timestamps
// never jump back. The WiFi delay only makes the robot clock late, so the largest offset of a window is the best
// estimation (the sample with the shortest delay)
void time_handler(z_loaned_sample_t *sample, void *arg)
{
    static double max_offset = -1e9;
    static uint8_t samples = 0;

    uint8_t buffer[16];
    z_bytes_reader_t reader = z_bytes_get_reader(z_sample_payload(sample));
    size_t len = z_bytes_reader_read(&reader, buffer, sizeof(buffer));

    double pc_time;
    if (!deserializeTime(buffer, len, &pc_time))
        return;

    double offset = pc_time - getPreciseTime();
    if (!clock_synced || fabs(offset) > 0.5) // first message or clock lost: set it directly
    {
        struct timeval tv = {(time_t)pc_time, (suseconds_t)((pc_time - (time_t)pc_time) * 1e6)};
        settimeofday(&tv, NULL);
        clock_synced = true;
        samples = 0;
        max_offset = -1e9;
        return;
    }

    max_offset = max(max_offset, offset);
    if (++samples >= 5)
    {
        struct timeval delta = {(time_t)floor(max_offset), (suseconds_t)((max_offset - floor(max_offset)) * 1e6)};
        adjtime(&delta, NULL);
        samples = 0;
        max_offset = -1e9;
    }
}

static void publishByte(z_owned_publisher_t *publisher, uint8_t value)
{
    uint8_t buffer[5];
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf(&payload, buffer, serializeByte(buffer, value));
    z_publisher_put(z_publisher_loan(publisher), z_bytes_move(&payload), NULL);
}

// shows the robot state in the on-board RGB led. Blinking = waiting for something
void statusLedTask(void *pvParameters)
{
    uint8_t last_r = 0, last_g = 0, last_b = 0;
    for (uint32_t tick = 0;; tick++) // 1 tick = 100 ms
    {
        RobotState state = getState();
        bool blink_slow = (state == STATE_WAIT_WIFI || state == STATE_CALIB_STILL);
        bool blink_fast = (state == STATE_CALIB_ROTATE);
        bool on = blink_slow ? (tick / 5) % 2 : (blink_fast ? tick % 2 : true);

        uint8_t level = on ? LED_BRIGHTNESS : 0;
        bool yellow = (state == STATE_READY && isDegraded());
        uint8_t r = (state == STATE_ERROR || yellow) ? level : 0;
        uint8_t b = (state == STATE_WAIT_WIFI || state == STATE_WAIT_ROS) ? level : 0;
        uint8_t g = (state >= STATE_CALIB_STILL) ? level : 0;

        if (r != last_r || g != last_g || b != last_b)
        {
            rgbLedWrite(RGB_BUILTIN, r, g, b);
            last_r = r;
            last_g = g;
            last_b = b;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void watchdogTask(void *pvParameters) // controles the wifi conection, 5 seconds check
{
    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            if (getState() == STATE_READY)
                setState(STATE_WAIT_WIFI);
            WiFi.disconnect();
            WiFi.begin(); // reconnect was not working
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        else
        {
            if (getState() == STATE_WAIT_WIFI)
                setState(STATE_READY); // zenoh-pico reconnects the session by itself

            vTaskDelay(whatchdog_delay);
        }
    }
}

void writeServosTask(void *pvParameters)
{
    float new_joints[4][3];
    for (;;)
    {
        if (xQueueReceive(Queues::joint_states_queue, &new_joints, portMAX_DELAY) == pdTRUE)
        {
            leg_lf.write(new_joints[0][0], new_joints[0][1], new_joints[0][2]);
            leg_rf.write(new_joints[1][0], new_joints[1][1], new_joints[1][2]);
            leg_lh.write(new_joints[2][0], new_joints[2][1], new_joints[2][2]);
            leg_rh.write(new_joints[3][0], new_joints[3][1], new_joints[3][2]);
        }
        // when arribes it write the position, with max port delay timeout
    }
}

// accel + gyro are always published. The magnetometer only while it gives data: if it is lost, imu/mag stops and
// imu/data_raw keeps going. Lost sensors are retried periodically. Any failure turns the led yellow
void readIMUTask(void *pvParameters)
{
    IMUdata current_data;
    uint8_t imu_fails = 0;
    uint32_t last_mag_ms = millis();
    uint32_t last_retry_ms = millis();
    TickType_t last_wake = xTaskGetTickCount();
    for (;;)
    {
        // calibration requested from ROS: gyro + accel (robot still), then magnetometer (rotate the robot).
        // Only with the robot ready and nobody commanding the joints (the robot must be picked up and rotated)
        if (imu_calibration_requested)
        {
            CalibrationResult result = CALIB_NO_IMU;
            if (getState() != STATE_READY || (command_received && millis() - last_command_ms < calib_idle_ms))
            {
                result = CALIB_BUSY;
            }
            else if (imu_ok)
            {
                setState(STATE_CALIB_STILL);
                imu.calibrate(true);
                result = CALIB_NO_MAG;
                if (mag_ok)
                {
                    setState(STATE_CALIB_ROTATE);
                    result = imu.calibrateMag() ? CALIB_OK : CALIB_MAG_REJECTED;
                }
                imu.saveCalibration();
                imu.printCalibration();
                setState(STATE_READY);
            }
            publishByte(&pub_calib_result, result);
            imu_calibration_requested = false;
            last_mag_ms = millis();
            last_wake = xTaskGetTickCount();
        }

        // try to recover lost sensors (resetting the MPU9250 also resets the magnetometer access)
        if ((!imu_ok || !mag_ok) && millis() - last_retry_ms > sensor_retry_ms)
        {
            last_retry_ms = millis();
            if (!imu_ok && imu.begin())
            {
                imu_ok = true;
                imu_fails = 0;
                Serial.println("IMU: accel/gyro recovered");
            }
            if (imu_ok && !mag_ok && imu.beginMag())
            {
                mag_ok = true;
                last_mag_ms = millis();
                Serial.println("IMU: magnetometer recovered");
            }
            last_wake = xTaskGetTickCount();
        }

        if (imu_ok)
        {
            if (imu.getAccelGyro(current_data.accel, current_data.gyro))
            {
                imu_fails = 0;

                current_data.mag_valid = false;
                if (mag_ok)
                {
                    if (imu.getMag(current_data.mag))
                    {
                        current_data.mag_valid = true;
                        last_mag_ms = millis();
                    }
                    else if (millis() - last_mag_ms > mag_timeout_ms)
                    {
                        mag_ok = false;
                        Serial.println("IMU: magnetometer lost, publishing only accel/gyro");
                    }
                }
                xQueueOverwrite(Queues::imu_data_queue, &current_data);
            }
            else if (++imu_fails >= imu_max_fails)
            {
                imu_ok = false;
                mag_ok = false;
                Serial.println("IMU: accel/gyro lost");
            }
        }

        setDegraded(!imu_ok || !mag_ok);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10)); // 100hz
    }
}

void sendStatusTask(void *pvParameters)
{
    IMUdata received_imu;
    static uint8_t imu_cdr_buffer[512]; // static: out of the task stack
    static uint8_t mag_cdr_buffer[256];
    uint32_t last_ready_ms = 0;
    for (;;)
    {
        // robot ready (2 Hz), also when there is no imu
        if (millis() - last_ready_ms >= 500)
        {
            last_ready_ms = millis();
            publishByte(&pub_ready, getState() == STATE_READY);
        }

        if (xQueueReceive(Queues::imu_data_queue, &received_imu, pdMS_TO_TICKS(100)) == pdTRUE) // if there is data in the buffer
        {
            // without the PC clock the timestamps would be from 1970 and jump when it arrives
            if (!clock_synced)
                continue;

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

            // same for magneitc field msg, only if there is a valid sample
            if (received_imu.mag_valid)
            {
                uint32_t mag_len = serializeMag(mag_cdr_buffer, sizeof(mag_cdr_buffer), received_imu, now);
                z_owned_bytes_t mag_payload;
                z_bytes_copy_from_buf(&mag_payload, mag_cdr_buffer, mag_len);
                if (z_publisher_put(z_publisher_loan(&pub_mag), z_bytes_move(&mag_payload), NULL) < 0)
                {
                    Serial.println("ZENOH: Error publishing MAG data");
                }
            }
        }
    }
}
