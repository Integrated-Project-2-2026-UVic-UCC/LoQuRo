#ifndef PCA9685_H
#define PCA9685_H

#include <math.h>
#include <Adafruit_PWMServoDriver.h>

namespace PCA9685Cfg
{
    static constexpr float freq = 100.0f; // 200Hz, T=0,005
    static constexpr float min_us = 500.0f;
    static constexpr float max_us = 2500.0f;

    static constexpr float period_us = 1000000.0f / freq;      // T = 5000.0 us
    static constexpr float abs_max_rads = M_PI;                // 270 degrees
    static constexpr float ticks_per_us = 4096.0f / period_us; // 0.8192 ticks/us
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