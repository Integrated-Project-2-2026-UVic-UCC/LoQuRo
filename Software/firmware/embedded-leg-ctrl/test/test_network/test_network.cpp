// Test de la conexion: WiFi guardado -> busca el bridge de ROS2 (scouting) -> sesion zenoh.
// No mueve motores. Necesita el bridge (zenoh-bridge-ros2dds) arrancado en el PC, en la misma red.

#include <Arduino.h>
#include <unity.h>

#include "config.h"
#include "utilities.h"
#include "zenoh.h"

static char locator[64] = "";
static bool session_ok = false;

void test_wifi_guardado()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(); // red guardada por el portal "LoQuRo"
    for (int i = 0; i < 150 && WiFi.status() != WL_CONNECTED; i++)
        delay(100);

    TEST_ASSERT_TRUE_MESSAGE(WiFi.status() == WL_CONNECTED, "no conecta al WiFi guardado (configuralo con el firmware normal, portal LoQuRo)");
    WiFi.setSleep(false);
    Serial.printf(">> WiFi %s  IP %s  RSSI %d dBm\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
    TEST_ASSERT_GREATER_THAN_INT_MESSAGE(-80, WiFi.RSSI(), "senal WiFi muy debil");
}

void test_bridge_encontrado()
{
    if (WiFi.status() != WL_CONNECTED)
        TEST_IGNORE_MESSAGE("sin WiFi");

    bool found = false;
    for (int i = 0; i < 5 && !found; i++)
        found = findRouter(locator, sizeof(locator));

    TEST_ASSERT_TRUE_MESSAGE(found, "no se encuentra el bridge por scouting (esta arrancado? misma red? multicast UDP 224.0.0.224:7446 bloqueado?)");
    Serial.printf(">> bridge encontrado en %s\n", locator);
    if (strncmp(locator, "udp", 3) != 0)
        Serial.println(">> AVISO: el bridge no escucha por UDP, se usa TCP. Arrancalo con: -l udp/0.0.0.0:7447 -l tcp/0.0.0.0:7447");
}

void test_sesion_zenoh()
{
    if (locator[0] == '\0')
        TEST_IGNORE_MESSAGE("bridge no encontrado");

    session_ok = initZenoh(locator);
    TEST_ASSERT_TRUE_MESSAGE(session_ok, "no se pudo abrir la sesion zenoh o declarar los topics");
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_wifi_guardado);
    RUN_TEST(test_bridge_encontrado);
    RUN_TEST(test_sesion_zenoh);
    UNITY_END();

    if (session_ok)
        z_session_drop(z_session_move(&s));
}

void loop() {}
