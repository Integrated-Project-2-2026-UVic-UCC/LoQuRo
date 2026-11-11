// IMPLEMENTATION OF IMUMPU9250 LIBRARY
#include "IMUMPU9250.h"

// calibration stored in non-volatile memory (gyro bias is not stored, it changes on every power-up)
struct IMUCalibration
{
    measure accel_offset;
    measure mag_offset;
    measure mag_scale;
};

IMUMPU9250::IMUMPU9250(uint8_t id, uint8_t mag_id)
{
    _id = id;
    _mag_id = mag_id;
};

bool IMUMPU9250::begin()
{
    // reset config
    this->write(_id, 0x6B, 0x80);
    delay(100);

    // Awake and configure clk
    this->write(_id, 0x6B, 0b001); // gyro x PLL, gyro y PLL, gyro z PLL (20Mhz aprox)

    uint8_t wia = 0;
    if (!this->read(_id, 0x75, 1, &wia) || wia != 0x71)
    {
        Serial.printf("IMU: Error I2C. MPU9250 WHO_AM_I returned: 0x%02X\n", wia);
        return false;
    }
    Serial.println("IMU: MPU9250 Connected sucessfully!");

    //  accel range
    this->write(_id, 0x1C, 0b10000); // scale: +-8g 0bxx000

    // gyro config
    this->write(_id, 0x1B, 0b01000); // scale: +500dps 0bxx000

    // DONE: Low pass filter 0x1A gyro 0x1D accel
    this->write(_id, 0x1A, 0x03); // Gyro LPF: 41Hz BW [8], bit 3 is inverted
    this->write(_id, 0x1D, 0x03); // Accel LPF: 44,8Hz BW [10], bit 3 is inverted
    return true;
};

bool IMUMPU9250::beginMag()
{
    // bypass mode AK8963
    this->write(_id, 0x37, 0x02);
    delay(10);

    uint8_t wai = 0;
    if (!this->read(_mag_id, 0x00, 1, &wai) || wai != 0x48)
    {
        Serial.printf("IMU: Error I2C. AK8963 WHO_AM_I returned: 0x%02X\n", wai);
        return false;
    }
    Serial.println("IMU: AK8963 Connected sucessfully!");

    // factory adjustment (fuse ROM). Mode changes must go through power-down
    this->write(_mag_id, 0x0A, 0x00); // power-down mode
    delay(10);
    this->write(_mag_id, 0x0A, 0b00001111); // fuse ROM acces mode
    delay(10);
    uint8_t adj_raw_data[3] = {128, 128, 128};
    this->read(_mag_id, 0x10, 3, adj_raw_data);
    this->write(_mag_id, 0x0A, 0x00); // power-down mode
    delay(10);

    _mag_adjustment = (measure){
        (float)((0.5 * (adj_raw_data[0] - 128.0)) / 128.0 + 1.0), // static cast for vble compatibility
        (float)((0.5 * (adj_raw_data[1] - 128.0)) / 128.0 + 1.0),
        (float)((0.5 * (adj_raw_data[2] - 128.0)) / 128.0 + 1.0),
    };

    // config
    //  Reg 0x0A del AK8963 (CNTL1) (PDF Pág 51)
    //  BIT [4] = 1 (Resolución 16 bits)
    //  MODE [3:0] = 0110 (Continuous measurement mode 2 -> 100Hz)
    //  Total = 0b00010110 = 0x16
    this->write(_mag_id, 0x0A, 0x16);
    delay(10);
    return true;
}

void IMUMPU9250::write(uint8_t id, uint8_t adr, uint8_t value)
{
    Wire.beginTransmission(id);
    Wire.write(adr);
    Wire.write(value);
    Wire.endTransmission();
}

bool IMUMPU9250::read(uint8_t id, uint8_t adr, int count, uint8_t *buffer)
{
    Wire.beginTransmission(id);
    Wire.write(adr);
    if (Wire.endTransmission(false) != 0)
        return false;

    if (Wire.requestFrom(id, (uint8_t)count, (uint8_t)true) != count) // request desired number of bytes
        return false;

    for (int i = 0; i < count; i++)
        buffer[i] = Wire.read();
    return true;
};

bool IMUMPU9250::getAccelGyro(measure &accel, measure &gyro)
{
    // accel (6) + temperature (2) + gyro (6) in a single transaction
    uint8_t raw_data[14];
    if (!this->read(_id, 0x3B, 14, raw_data))
        return false;

    int16_t raw_ax = (raw_data[0] << 8) | raw_data[1];
    int16_t raw_ay = (raw_data[2] << 8) | raw_data[3];
    int16_t raw_az = (raw_data[4] << 8) | raw_data[5];
    int16_t raw_gx = (raw_data[8] << 8) | raw_data[9];
    int16_t raw_gy = (raw_data[10] << 8) | raw_data[11];
    int16_t raw_gz = (raw_data[12] << 8) | raw_data[13];

    // 8g scale
    accel.x = raw_ax / 4096.0 - _accel_offset.x;
    accel.y = raw_ay / 4096.0 - _accel_offset.y;
    accel.z = raw_az / 4096.0 - _accel_offset.z;

    // 500dps scale
    gyro.x = raw_gx / 65.5 - _gyro_offset.x;
    gyro.y = raw_gy / 65.5 - _gyro_offset.y;
    gyro.z = raw_gz / 65.5 - _gyro_offset.z;

    return true;
};

bool IMUMPU9250::getMag(measure &mag)
{
    // ST1 + 6 data bytes + ST2 (reading ST2 releases the next sample)
    uint8_t raw_data[8];

    // no new data (DRDY) or magnetic overflow (HOFL)
    if (!this->read(_mag_id, 0x02, 8, raw_data) || !(raw_data[0] & 0b1) || (raw_data[7] & 0b1000))
        return false;

    // decodification
    int16_t raw_x = (raw_data[2] << 8) | raw_data[1]; // little endian, LSB first
    int16_t raw_y = (raw_data[4] << 8) | raw_data[3];
    int16_t raw_z = (raw_data[6] << 8) | raw_data[5];

    // apply calibration: (raw*factory_adjustment*conversion_factor-measured_offset)*measured_scale
    mag.x = ((raw_y * _mag_adjustment.y * IMUCfg::res) - _mag_offset.x) * _mag_scale.x;
    mag.y = ((raw_x * _mag_adjustment.x * IMUCfg::res) - _mag_offset.y) * _mag_scale.y;
    mag.z = ((-raw_z * _mag_adjustment.z * IMUCfg::res) - _mag_offset.z) * _mag_scale.z; // in the sensor z is inverted

    return true;
};

bool IMUMPU9250::waitStill()
{
    uint32_t start = millis();
    while (millis() - start < IMUCfg::still_timeout)
    {
        // gyro variation (max - min) in a window of 0.5 s, it does not depend on the bias
        measure a, g, min_g = {1e6, 1e6, 1e6}, max_g = {-1e6, -1e6, -1e6};
        for (int i = 0; i < 50; i++)
        {
            if (this->getAccelGyro(a, g))
            {
                min_g = {min(min_g.x, g.x), min(min_g.y, g.y), min(min_g.z, g.z)};
                max_g = {max(max_g.x, g.x), max(max_g.y, g.y), max(max_g.z, g.z)};
            }
            delay(10);
        }
        if (max_g.x - min_g.x < IMUCfg::still_threshold &&
            max_g.y - min_g.y < IMUCfg::still_threshold &&
            max_g.z - min_g.z < IMUCfg::still_threshold)
            return true;
    }
    Serial.println("IMU: The sensor is still moving, calibrating anyway");
    return false;
}

void IMUMPU9250::calibrate(bool with_accel)
{
    this->waitStill();

    _gyro_offset = {0, 0, 0};
    if (with_accel)
        _accel_offset = {0, 0, 0};

    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    int valid = 0;

    Serial.println("IMU: calibrating sensor... Please do not move and place it correctly");

    for (int i = 0; i < IMUCfg::samples; i++)
    {
        measure a, g;
        if (this->getAccelGyro(a, g))
        {
            ax += a.x;
            ay += a.y;
            az += a.z;
            gx += g.x;
            gy += g.y;
            gz += g.z;
            valid++;
        }
        delay(10); // simulate lectures at 100hz in 2 second due to sample number
    }
    if (valid == 0)
    {
        Serial.println("IMU: Calibration failed, no data");
        return;
    }

    if (with_accel)
    {
        _accel_offset.x = ax / valid;
        _accel_offset.y = ay / valid;
        // at rest the z axis measures 1 g: +1 g with z up, -1 g with z down (as mounted in the robot)
        _accel_offset.z = (az / valid) - copysignf(1.0f, az);
    }

    _gyro_offset.x = gx / valid;
    _gyro_offset.y = gy / valid;
    _gyro_offset.z = gz / valid;

    Serial.println("IMU: Calibration completed");
};

bool IMUMPU9250::calibrateMag()
{
    const measure old_offset = _mag_offset;
    const measure old_scale = _mag_scale;
    _mag_offset = (measure){0, 0, 0};
    _mag_scale = (measure){1, 1, 1};

    Serial.println("IMU: Mag calibration, rotate the robot in all directions (8-figure pattern)...");

    float max_x = -10000.0, max_y = -10000.0, max_z = -10000.0;
    float min_x = 10000.0, min_y = 10000.0, min_z = 10000.0;

    for (int i = 0; i < IMUCfg::mag_samples; i++)
    {
        measure m;
        if (!this->getMag(m))
        {
            delay(10);
            continue;
        }
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

        delay(10); // 100hz
    }

    float delta_x = max_x - min_x;
    float delta_y = max_y - min_y;
    float delta_z = max_z - min_z;

    // the sensor was not rotated enough: keep the previous calibration
    if (delta_x < IMUCfg::mag_min_delta || delta_y < IMUCfg::mag_min_delta || delta_z < IMUCfg::mag_min_delta)
    {
        _mag_offset = old_offset;
        _mag_scale = old_scale;
        Serial.println("IMU: Mag calibration rejected (not enough rotation), keeping previous values");
        return false;
    }

    // hard iron (offset)
    _mag_offset.x = (max_x + min_x) / 2.0;
    _mag_offset.y = (max_y + min_y) / 2.0;
    _mag_offset.z = (max_z + min_z) / 2.0;
    // soft iron (bias)
    float avg_delta = (delta_x + delta_y + delta_z) / 3.0;

    _mag_scale.x = avg_delta / delta_x;
    _mag_scale.y = avg_delta / delta_y;
    _mag_scale.z = avg_delta / delta_z;

    Serial.println("IMU: Mag calibration completed");
    return true;
}

bool IMUMPU9250::loadCalibration()
{
    IMUCalibration calib;
    Preferences preferences;
    preferences.begin("imu", true);
    size_t len = preferences.getBytes("calib", &calib, sizeof(calib));
    preferences.end();

    if (len != sizeof(calib))
        return false;

    _accel_offset = calib.accel_offset;
    _mag_offset = calib.mag_offset;
    _mag_scale = calib.mag_scale;
    return true;
}

void IMUMPU9250::printCalibration()
{
    Serial.printf("IMU: accel offset %.3f %.3f %.3f g | mag offset %.1f %.1f %.1f uT | mag scale %.2f %.2f %.2f\n",
                  _accel_offset.x, _accel_offset.y, _accel_offset.z, _mag_offset.x, _mag_offset.y, _mag_offset.z,
                  _mag_scale.x, _mag_scale.y, _mag_scale.z);
}

void IMUMPU9250::saveCalibration()
{
    IMUCalibration calib = {_accel_offset, _mag_offset, _mag_scale};
    Preferences preferences;
    preferences.begin("imu", false);
    preferences.putBytes("calib", &calib, sizeof(calib));
    preferences.end();
}
