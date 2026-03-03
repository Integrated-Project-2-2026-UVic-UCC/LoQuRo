// SERVO LIBRARY
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

#endif