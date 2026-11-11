// Mide en el robot cada cuanto llegan los comandos de /joint_commands. NO mueve motores.
//
// Uso: launch + una rutina en marcha en el PC (ros2 run loquro_control routine_player ...), luego
//   pio test -e esp32-s3-devkitc-1 -f test_command_timing --without-testing
//   pio device monitor
// Cada 5 s imprime el intervalo entre comandos. Con la rutina a 50 Hz lo ideal es ~20 ms siempre.
//  - Intervalos de ~0 ms seguidos de huecos largos = llegan a rafagas (la red los agrupa): el robot solo aplica el
//    ultimo de cada rafaga y la pierna va a saltos.
//  - Perdidos = comandos que no llegan (hueco > 1.5 periodos).

#include <Arduino.h>

#include "config.h"
#include "utilities.h"
#include "zenoh.h"

static z_owned_subscriber_t sub_timing;
static volatile uint32_t last_us = 0;
static volatile uint32_t count = 0, burst = 0, lost = 0, max_gap_us = 0;
static volatile uint32_t hist[6] = {0}; // <5, 5-15, 15-25, 25-40, 40-100, >100 ms

static void on_command(z_loaned_sample_t *sample, void *arg)
{
    uint32_t now = micros();
    if (last_us != 0)
    {
        uint32_t gap = now - last_us;
        uint32_t ms = gap / 1000;
        hist[ms < 5 ? 0 : ms < 15 ? 1 : ms < 25 ? 2 : ms < 40 ? 3 : ms < 100 ? 4 : 5]++;
        if (ms < 5)
            burst++;
        if (gap > 30000) // > 1.5 periods of 20 ms
            lost += gap / 20000 - 1;
        if (gap > max_gap_us)
            max_gap_us = gap;
    }
    last_us = now;
    count++;
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    WiFi.mode(WIFI_STA);
    WiFi.begin(); // red guardada por el portal "LoQuRo"
    while (WiFi.status() != WL_CONNECTED)
        delay(100);
    WiFi.setSleep(false);
    Serial.printf("WiFi %s  IP %s  RSSI %d dBm\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());

    char locator[64];
    Serial.println("Buscando el bridge... (launch en marcha y PC en la misma WiFi que el robot)");
    while (!findRouter(locator, sizeof(locator)) || !initZenoh(locator))
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.printf("Bridge en %s\n", locator);

    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, on_command, NULL, NULL);
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
    z_declare_subscriber(z_session_loan(&s), &sub_timing, z_view_keyexpr_loan(&ke), z_closure_sample_move(&callback), NULL);
    Serial.println("Midiendo... lanza una rutina en el PC");
}

// publishes robot/ready = true like the firmware, otherwise routine_player does not send commands
static void publishReady()
{
    uint8_t buffer[5];
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf(&payload, buffer, serializeByte(buffer, 1));
    z_publisher_put(z_publisher_loan(&pub_ready), z_bytes_move(&payload), NULL);
}

void loop()
{
    for (int i = 0; i < 10; i++) // 5 s, robot/ready at 2 Hz
    {
        publishReady();
        delay(500);
    }
    uint32_t n = count;
    count = 0;
    Serial.printf("\n%lu comandos en 5 s (%.1f Hz)  RSSI %d dBm\n", n, n / 5.0f, WiFi.RSSI());
    if (n < 2)
        return;
    Serial.printf("  intervalo: <5ms %lu | 5-15 %lu | 15-25 %lu | 25-40 %lu | 40-100 %lu | >100ms %lu\n",
                  hist[0], hist[1], hist[2], hist[3], hist[4], hist[5]);
    Serial.printf("  rafagas (<5 ms): %lu   perdidos aprox: %lu   hueco maximo: %lu ms\n", burst, lost, max_gap_us / 1000);
    for (auto &h : hist)
        h = 0;
    burst = lost = max_gap_us = 0;
}
