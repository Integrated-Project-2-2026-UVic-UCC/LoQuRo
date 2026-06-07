// test multiplexor/TCA9548A.cpp
#include <Arduino.h>
#include <unity.h>
#include "Wire.h"

// ── Lógica original ──────────────────────────────────────────
const byte start_address = 8;
const byte end_address = 119;

static int dispositivos_encontrados = 0;
static bool direccion_encontrada[128] = {false};

void scanI2CBus(byte from_addr, byte to_addr)
{
    byte data = 0;
    for (byte addr = from_addr; addr <= to_addr; addr++)
    {
        Wire.beginTransmission(addr);
        byte rc = Wire.endTransmission();
        if (rc == 0)
        {
            dispositivos_encontrados++;
            direccion_encontrada[addr] = true;
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────

// Verifica que al menos un dispositivo responde en el bus
void test_hay_dispositivos_en_bus()
{
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, dispositivos_encontrados,
                                     "No se encontro ningun dispositivo I2C en el bus");
}

// Verifica que el TCA9548A responde en su dirección esperada
void test_tca9548a_responde()
{
    TEST_ASSERT_TRUE_MESSAGE(direccion_encontrada[0x70],
                             "TCA9548A no encontrado en 0x70");
}

// Verifica que direcciones reservadas (<8) no dieron falsos positivos
void test_sin_dispositivos_en_rango_reservado()
{
    for (byte addr = 0; addr < start_address; addr++)
    {
        TEST_ASSERT_FALSE(direccion_encontrada[addr]);
    }
}

// ── Entrypoint ────────────────────────────────────────────────
void setup()
{
    delay(2000);
    Wire.begin();

    // Escaneo único antes de correr los tests
    scanI2CBus(start_address, end_address);

    UNITY_BEGIN();
    RUN_TEST(test_hay_dispositivos_en_bus);
    RUN_TEST(test_tca9548a_responde);
    RUN_TEST(test_sin_dispositivos_en_rango_reservado);
    UNITY_END();
}

void loop() {}