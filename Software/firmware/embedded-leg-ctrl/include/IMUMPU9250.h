// IMU LIBRARY
#ifndef _IMUMPU9250_H_
#define _IMUMPU9250_H_
#include <Wire.h>

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
};

namespace Params
{
    static constexpr float res = 0.15;
    static constexpr uint8_t samples = 200;
    static constexpr uint16_t mag_samples = 1000;
};

class IMUMPU9250
{
public:
    IMUMPU9250(uint8_t id, uint8_t mag_id);
    void begin();
    void beginMag();
    void read(uint8_t id, uint8_t adr, int count, uint8_t *buffer);
    void write(uint8_t id, uint8_t adr, uint8_t value);
    void calibrate();
    void calibrateMag();
    measure getAccel();
    measure getGyro();
    measure getMag();

private:
    uint8_t _id;
    uint8_t _mag_id;
    measure _accel_offset;
    measure _gyro_offset;
    measure _mag_adjustment;
    measure _mag_offset;
    measure _mag_scale;
};
#endif
