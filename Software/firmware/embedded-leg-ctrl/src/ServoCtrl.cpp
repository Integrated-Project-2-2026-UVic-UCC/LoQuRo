// IMPLEMENTATION OF SERVO LIBRARY
#include "ServoCtrl.h"

ServoCtrl::ServoCtrl(int pin, int channel, int resolution)
{
    _pin = pin;
    _channel = channel;
    _resolution = resolution;
    _max_rads = ServoCfg::abs_max_rads; // robot maximum range is 270 //0.09 margin
    ledcSetup(_channel, ServoCfg::freq, _resolution);
    ledcAttachPin(_pin, _channel);
}

void ServoCtrl::setLimit(float max_rads)
{
    _max_rads = max_rads;
}

void ServoCtrl::write(float rads)
{
    if (rads > _max_rads)
        rads = _max_rads;
    if (rads < 0)
        rads = 0;

    // TODO: añadir offset por rangos

    // FÓRMULA CORREGIDA:
    // Calculamos el pulso basándonos en que el máximo recorrido es 270 grados
    float pulse = ServoCfg::min_us + (rads / ServoCfg::abs_max_rads) * (ServoCfg::max_us - ServoCfg::min_us);

    float duty = pulse / ServoCfg::period_us;

    // Cálculo a 16 bits seguro
    uint16_t max_duty_val = (1 << _resolution) - 1;
    uint16_t duty_bits = (uint32_t)(duty * max_duty_val);

    ledcWrite(_channel, duty_bits);
}

void ServoCtrl::writeDegrees(float degrees)
{
    write(degrees * (M_PI / 180.0));
}