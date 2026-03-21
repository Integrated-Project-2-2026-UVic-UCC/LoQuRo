// IMPLEMENTATION OF IMUMPU9250 LIBRARY
#include "IMUMPU9250.h"
#include "config.h"

IMUMPU9250::IMUMPU9250(uint8_t id, uint8_t mag_id)
{
    _id = id;
    _mag_id = mag_id;
};
void IMUMPU9250::begin()
{
    Wire.begin(8, 9);
    Wire.setClock(400000); // bus freq

    // reset config
    this->write(_id, 0x6B, 0x80);
    delay(100);

    // Awake and configure clk
    this->write(_id, 0x6B, 0b001); // gyro x PLL, gyro y PLL, gyro z PLL (20Mhz aprox)

    uint8_t wia;
    Wire.beginTransmission(_id);
    Wire.write(0x75);
    Wire.endTransmission(false);

    Wire.requestFrom(_id, (uint8_t)1, (uint8_t)true);

    wia = Wire.read();

    wia == 0x71 ? Serial.println("IMU: MPU9250 Connected sucessfully!") : Serial.print("IMU: Error I2C. MPU9250 WHO_AM_I returned: 0x");
    Serial.println(wia, HEX);

    //  accel range
    this->write(_id, 0x1C, 0b10000); // scale: +-8g 0bxx000

    // gyro config
    this->write(_id, 0x1B, 0b01000); // scale: +500dps 0bxx000

    // DONE: Low pass filter 0x1A gyro 0x1D accel
    this->write(_id, 0x1A, 0x03); // Gyro LPF: 41Hz BW [8], bit 3 is inverted
    this->write(_id, 0x1D, 0x03); // Accel LPF: 44,8Hz BW [10], bit 3 is inverted
};

void IMUMPU9250::beginMag()
{
    // bypass mode AK8963
    this->write(_id, 0x37, 0x02);
    // config
    //  Reg 0x0A del AK8963 (CNTL1) (PDF Pág 51)
    //  BIT [4] = 1 (Resolución 16 bits)
    //  MODE [3:0] = 0110 (Continuous measurement mode 2 -> 100Hz)
    //  Total = 0b00010110 = 0x16
    this->write(_mag_id, 0x0A, 0x16);
    delay(10);

    uint8_t wai;
    Wire.beginTransmission(_mag_id);
    Wire.write(0x00);
    Wire.endTransmission(false);

    Wire.requestFrom(_mag_id, (uint8_t)1, (uint8_t)true);

    wai = Wire.read();

    wai == 0x48 ? Serial.println("IMU: AK8963 Connected sucessfully!") : Serial.print("IMU: Error I2C. AK8963 WHO_AM_I returned: 0x");
    Serial.println(wai, HEX);
}

void IMUMPU9250::write(uint8_t id, uint8_t adr, uint8_t value)
{
    Wire.beginTransmission(id);
    Wire.write(adr);
    Wire.write(value); // scale: +100dps 0bxx000
    Wire.endTransmission();
}

void IMUMPU9250::read(uint8_t id, uint8_t adr, int count, uint8_t *buffer)
{
    Wire.beginTransmission(id);
    Wire.write(adr);
    Wire.endTransmission(false);

    Wire.requestFrom(id, (uint8_t)count, (uint8_t)true); // request desired number of bytes
    for (int i = 0; i < count; i++)
    {
        if (Wire.available())
        {
            buffer[i] = Wire.read();
        }
    }
};

measure IMUMPU9250::getAccel()
{
    measure accel;
    uint8_t raw_data[6];

    this->read(_id, 0x3B, 6, raw_data);

    int16_t raw_x = (raw_data[0] << 8) | raw_data[1];
    int16_t raw_y = (raw_data[2] << 8) | raw_data[3];
    int16_t raw_z = (raw_data[4] << 8) | raw_data[5];

    // 8g scale
    accel.x = raw_x / 4096.0 - _accel_offset.x;
    accel.y = raw_y / 4096.0 - _accel_offset.y;
    accel.z = raw_z / 4096.0 - _accel_offset.z;

    return accel;
};

measure IMUMPU9250::getGyro()
{

    measure gyro;
    uint8_t raw_data[6];

    this->read(_id, 0x43, 6, raw_data);

    int16_t raw_x = (raw_data[0] << 8) | raw_data[1];
    int16_t raw_y = (raw_data[2] << 8) | raw_data[3];
    int16_t raw_z = (raw_data[4] << 8) | raw_data[5];

    // 500dps scale
    gyro.x = raw_x / 65.5 - _gyro_offset.x;
    gyro.y = raw_y / 65.5 - _gyro_offset.y;
    gyro.z = raw_z / 65.5 - _gyro_offset.z;

    return gyro;
};
measure IMUMPU9250::getMag()
{
    measure mag;
    uint8_t raw_data[7];

    this->read(_mag_id, 0x03, 7, raw_data);

    if (raw_data[6] & 0b1000) // check HOFL
    {
        Serial.println("IMU: AK8963 Overflow");
        return (measure){0.0, 0.0, 0.0};
    }
    if (!(raw_data[6] & 0b10000)) // check BITM
    {
        Serial.println("IMU: Resolution incorrect");
        return (measure){0.0, 0.0, 0.0};
    }
    // decodification
    int16_t raw_x = (raw_data[1] << 8) | raw_data[0]; // little endian, LSB first
    int16_t raw_y = (raw_data[3] << 8) | raw_data[2];
    int16_t raw_z = (raw_data[5] << 8) | raw_data[4];

    // apply calibration: (raw*factory_adjustment*conversion_factor-measured_offset)*measured_scale
    mag.x = ((raw_y * _mag_adjustment.y * Params::res) - _mag_offset.x) * _mag_scale.x;
    mag.y = ((raw_x * _mag_adjustment.x * Params::res) - _mag_offset.y) * _mag_scale.y;
    mag.z = ((-raw_z * _mag_adjustment.z * Params::res) - _mag_offset.z) * _mag_scale.z;

    return mag;
};

void IMUMPU9250::calibrate()
{
    _accel_offset = {0, 0, 0};
    _gyro_offset = {0, 0, 0};

    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

    Serial.println("IMU: calibrating sensor... Please do not move and place it correctly");

    for (int i = 0; i < Params::samples; i++)
    {
        measure a = this->getAccel();
        measure g = this->getGyro();
        ax += a.x;
        ay += a.y;
        az += a.z;
        gx += g.x;
        gy += g.y;
        gz += g.z;
        delay(10); // simulate lectures at 100hz in 2 second due to sample number
    }

    _accel_offset.x = ax / Params::samples;
    _accel_offset.y = ay / Params::samples;
    _accel_offset.z = (az / Params::samples) - 1.0f; // if is static, accel will be 1g in z

    _gyro_offset.x = gx / Params::samples;
    _gyro_offset.y = gy / Params::samples;
    _gyro_offset.z = gz / Params::samples;

    Serial.println("IMU: Calibration completed");
};

void IMUMPU9250::calibrateMag()
{
    this->write(_mag_id, 0x0A, 0b00001111); // fuse ROM acces mode
    uint8_t adj_raw_data[3];
    this->read(_mag_id, 0x10, 3, adj_raw_data);
    this->write(_mag_id, 0x0A, 0); // power-down mode

    delayMicroseconds(100);

    _mag_adjustment = (measure){
        // factory adjustmen
        (float)((0.5 * (adj_raw_data[0] - 128.0)) / 128.0 + 1.0), // static cast for vble compatibility
        (float)((0.5 * (adj_raw_data[1] - 128.0)) / 128.0 + 1.0),
        (float)((0.5 * (adj_raw_data[2] - 128.0)) / 128.0 + 1.0),
    };

    this->write(_mag_id, 0x0A, 0x16); // return to measuremnet mode
    delay(10);

    _mag_offset = (measure){0, 0, 0};
    _mag_scale = (measure){1, 1, 1};

    // Wait for movement, otherwise calibration wont be correct
    Serial.println("IMU: Mag calibration ready. WAITING FOR SENSOR MOVEMENT...");
    measure start_mag = this->getMag();
    while (true)
    {
        measure curr_mag = this->getMag();
        // Calculamos la diferencia absoluta total
        float diff = abs(curr_mag.x - start_mag.x) +
                     abs(curr_mag.y - start_mag.y) +
                     abs(curr_mag.z - start_mag.z);

        // Si el campo magnético cambia más de 15 uT, es que lo estás rotando
        if (diff > 15.0)
            break;

        delay(20);
    }
    Serial.println("IMU: Movement detected! Rotate in an 8-figure pattern...");

    float max_x = -10000.0, max_y = -10000.0, max_z = -10000.0;
    float min_x = 10000.0, min_y = 10000.0, min_z = 10000.0;

    for (int i = 0; i < Params::mag_samples; i++)
    {
        measure m = this->getMag();
        if (m.x > max_x)
            max_x = m.x;
        if (m.x < min_x)
            min_x = m.x;
        if (m.y > max_y)
            max_y = m.y;
        if (m.y < min_y)
            min_y = m.y;
        if (m.z > max_z)
            max_z = m.z;
        if (m.z < min_z)
            min_z = m.z;

        delay(10); // simulate lectures at 100hz in 2 second due to sample number
    }

    // hard iron (offset)
    _mag_offset.x = (max_x + min_x) / 2.0;
    _mag_offset.y = (max_y + min_y) / 2.0;
    _mag_offset.z = (max_z + min_z) / 2.0;
    // soft iron (bias)
    float delta_x = max_x - min_x;
    float delta_y = max_y - min_y;
    float delta_z = max_z - min_z;
    float avg_delta = (delta_x + delta_y + delta_z) / 3.0;

    _mag_scale.x = avg_delta / delta_x;
    _mag_scale.y = avg_delta / delta_y;
    _mag_scale.z = avg_delta / delta_z;
}