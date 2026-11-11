// Test de la IMU (MPU9250 + AK8963). No mueve motores ni usa la red.
// El robot tiene que estar QUIETO durante el test.

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

#include "hardware.h" // imu del código real

static bool imu_ok = false;
static bool mag_ok = false;

#define REQUIRE(ok) \
    if (!(ok))      \
    TEST_IGNORE_MESSAGE("sensor no detectado")

void test_mpu9250_responde()
{
    imu_ok = imu.begin();
    TEST_ASSERT_TRUE_MESSAGE(imu_ok, "MPU9250 no responde en 0x68 (Wire SDA=8 SCL=9)");
}

void test_ak8963_responde()
{
    REQUIRE(imu_ok);
    mag_ok = imu.beginMag();
    TEST_ASSERT_TRUE_MESSAGE(mag_ok, "AK8963 (magnetometro) no responde en 0x0C");
}

void test_calibracion_guardada()
{
    REQUIRE(imu_ok);
    if (!imu.loadCalibration())
        TEST_IGNORE_MESSAGE("no hay calibracion guardada: usa la accion /calibrate_imu con el firmware normal");
    imu.printCalibration();
}

// en reposo el acelerometro mide 1 g y el giroscopio casi 0
void test_accel_gyro_en_reposo()
{
    REQUIRE(imu_ok);
    measure a, g;
    float norm = 0, gx = 0, gy = 0, gz = 0;
    const int n = 100;
    for (int i = 0; i < n; i++)
    {
        TEST_ASSERT_TRUE_MESSAGE(imu.getAccelGyro(a, g), "fallo de lectura I2C");
        norm += sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        gx += g.x;
        gy += g.y;
        gz += g.z;
        delay(10);
    }
    Serial.printf(">> accel |a| = %.3f g   gyro medio = %.2f %.2f %.2f dps (gyro sin calibrar)\n", norm / n, gx / n, gy / n, gz / n);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.15f, 1.0f, norm / n, "el modulo de la aceleracion no es 1 g");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(10.0f, 0.0f, gx / n, "gyro x lejos de 0 (se esta moviendo?)");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(10.0f, 0.0f, gy / n, "gyro y lejos de 0 (se esta moviendo?)");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(10.0f, 0.0f, gz / n, "gyro z lejos de 0 (se esta moviendo?)");
}

// el campo magnetico terrestre esta entre 25 y 65 uT
void test_magnetometro()
{
    REQUIRE(mag_ok);
    measure m;
    bool got = false;
    for (int i = 0; i < 20 && !got; i++) // el magnetometro da una muestra cada 10 ms
    {
        delay(10);
        got = imu.getMag(m);
    }
    TEST_ASSERT_TRUE_MESSAGE(got, "el magnetometro no da muestras nuevas");
    float norm = sqrtf(m.x * m.x + m.y * m.y + m.z * m.z);
    Serial.printf(">> mag = %.1f %.1f %.1f uT   |m| = %.1f uT\n", m.x, m.y, m.z, norm);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(90.0f, 100.0f, norm, "campo magnetico fuera de rango (10-190 uT)");
}

// una lectura completa tiene que caber de sobra en el periodo de 10 ms (100 Hz)
void test_tiempo_de_lectura()
{
    REQUIRE(imu_ok);
    measure a, g, m;
    const int n = 100;
    uint32_t t0 = micros();
    for (int i = 0; i < n; i++)
    {
        imu.getAccelGyro(a, g);
        imu.getMag(m);
    }
    uint32_t us = (micros() - t0) / n;
    Serial.printf(">> lectura completa (accel + gyro + mag): %lu us\n", us);
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(5000, us, "la lectura I2C tarda mas de 5 ms");
}

// muestra unas cuantas lecturas para verlas a ojo
void test_mostrar_lecturas()
{
    REQUIRE(imu_ok);
    measure a, g, m = {0, 0, 0};
    for (int i = 0; i < 10; i++)
    {
        imu.getAccelGyro(a, g);
        imu.getMag(m);
        Serial.printf(">> ACCEL %6.2f %6.2f %6.2f g | GYRO %7.2f %7.2f %7.2f dps | MAG %6.1f %6.1f %6.1f uT\n",
                      a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z);
        delay(100);
    }
    TEST_PASS();
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Wire.end();
    Wire.begin(8, 9);
    Wire.setClock(400000);

    UNITY_BEGIN();
    RUN_TEST(test_mpu9250_responde);
    RUN_TEST(test_ak8963_responde);
    RUN_TEST(test_calibracion_guardada);
    RUN_TEST(test_accel_gyro_en_reposo);
    RUN_TEST(test_magnetometro);
    RUN_TEST(test_tiempo_de_lectura);
    RUN_TEST(test_mostrar_lecturas);
    UNITY_END();
}

void loop() {}
