#ifndef SERVO_CTRL_H
#define SERVO_CTRL_H
#include <Arduino.h>
#include <math.h>

namespace ServoCfg // constexpr: se evalua en tiempo de compliacion, no ocupa RAM, no cambian en ejecucion
{                  // static: en librerias, al no cambiar en cada definicion, se comparte entre todas las instancias, no ocupa RAM extra por cada instancia
    static constexpr float freq = 50.0;
    static constexpr float min_us = 500.0;
    static constexpr float max_us = 2500.0;
    static constexpr float period_us = 20000.0;               // 50 Hz
    static constexpr float abs_max_rads = (3.0 * M_PI) / 2.0; // 270 grados
}

class ServoCtrl
{
public:
    ServoCtrl(int pin, int channel, int resolution);
    void write(float rads);
    void writeDegrees(float degrees);
    void setLimit(float max_rads); // method to set maximum values to no break the leg

private:
    int _pin;
    int _channel;
    int _resolution;
    float _max_rads;
};

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
#endif