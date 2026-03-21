#include "PCA9685.h"

PCA9685Servo::PCA9685Servo(Adafruit_PWMServoDriver *driver, uint8_t channel)
{
    _driver = driver;
    _channel = channel;
    _max_rads = PCA9685Cfg::abs_max_rads;
}

void PCA9685Servo::setLimit(float max_rads)
{
    _max_rads = max_rads;
}

void PCA9685Servo::write(float rads)
{
    // add limits
    if (rads > _max_rads)
        rads = _max_rads;
    if (rads < 0.0f)
        rads = 0.0f;

    // FEATURE: add offset for ranges if necessary

    // map radians to us
    float pulse_us = PCA9685Cfg::min_us + (rads / PCA9685Cfg::abs_max_rads) * (PCA9685Cfg::max_us - PCA9685Cfg::min_us);

    // us to ticks
    uint16_t duty_ticks = (uint16_t)(pulse_us * PCA9685Cfg::ticks_per_us);

    // write angle to servo using adafruit library
    _driver->setPWM(_channel, 0, duty_ticks);
}

void PCA9685Servo::writeDegrees(float degrees)
{
    write(degrees * (M_PI / 180.0f));
}