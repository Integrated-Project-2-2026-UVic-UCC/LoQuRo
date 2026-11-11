// Servos wrap library for PCA9685, adapted from Adafruit_PWMServoDriver
#ifndef PCA9685_H
#define PCA9685_H

#include <math.h>
#include <Adafruit_PWMServoDriver.h>

namespace PCA9685Cfg
{
    static constexpr float freq = 100.0f; // 100Hz, T=0,01
    static constexpr float min_us = 500.0f;
    static constexpr float max_us = 2500.0f;

    static constexpr float period_us = 1000000.0f / freq;      // T = 10000.0 us
    static constexpr float abs_max_rads = M_PI;                // commanded angle at max_us: 500-2500 us is mapped to 0-PI rad
    static constexpr float ticks_per_us = 4096.0f / period_us; // 0.4096 ticks/us
}

class PCA9685Servo
{
public:
    PCA9685Servo(Adafruit_PWMServoDriver *driver, uint8_t channel);

    void write(float rads);
    void writeDegrees(float degrees);
    void setLimit(float max_rads);

private:
    Adafruit_PWMServoDriver *_driver;
    uint8_t _channel;
    float _max_rads;
};

#endif