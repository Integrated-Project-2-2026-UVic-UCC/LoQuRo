// IMU LIBRARY
#ifndef _IMUMPU9250_H_
#define _IMUMPU9250_H_
#include "config.h"

struct measure
{
    float x;
    float y;
    float z;
};

struct IMUdata
{
    measure accel;
    measure gyro;
    measure mag;
    bool mag_valid; // false = magnetometer not available, mag is not published
};

namespace IMUCfg
{
    static constexpr float res = 0.15;
    static constexpr uint8_t samples = 200;        // 2 s at 100 Hz (gyro / accel calibration)
    static constexpr uint16_t mag_samples = 2000;  // 20 s at 100 Hz (mag calibration window)
    static constexpr float mag_min_delta = 20.0f;  // uT, minimum range per axis to accept a mag calibration
    static constexpr float still_threshold = 2.0f; // dps, max gyro variation in 0.5 s to consider the sensor at rest
    static constexpr uint32_t still_timeout = 10000; // ms, max time waiting for the sensor to be at rest
};

class IMUMPU9250
{
public:
    IMUMPU9250(uint8_t id, uint8_t mag_id);
    bool begin();    // false if the MPU9250 does not answer
    bool beginMag(); // false if the AK8963 does not answer
    bool read(uint8_t id, uint8_t adr, int count, uint8_t *buffer);
    void write(uint8_t id, uint8_t adr, uint8_t value);
    bool waitStill();                // waits until the sensor is at rest. false = timeout
    void calibrate(bool with_accel); // waits until still, then gyro bias (always) + accel offset (optional)
    bool calibrateMag();             // rotate the sensor during the window. false = rejected, old values kept
    bool loadCalibration();          // accel + mag calibration from non-volatile memory
    void saveCalibration();
    void printCalibration();         // prints the calibration in use by the serial port
    bool getAccelGyro(measure &accel, measure &gyro); // single burst read, same sample instant
    bool getMag(measure &mag);                         // false = no new valid sample (not ready, overflow or I2C error)

private:
    uint8_t _id;
    uint8_t _mag_id;
    measure _accel_offset = {0, 0, 0};
    measure _gyro_offset = {0, 0, 0};
    measure _mag_adjustment = {1, 1, 1};
    measure _mag_offset = {0, 0, 0};
    measure _mag_scale = {1, 1, 1};
};
#endif
