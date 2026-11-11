// Test de piernas: solo inicializa Wire1 + PCA9685 (sin red, sin IMU, sin encoders).
//
// Uso: activa/desactiva piernas en la lista LEGS de abajo (true/false) y pulsa el icono de Test.
// Convención del código: lf = front-left, rf = front-right, lh = rear-left, rh = rear-right.
// Reutiliza Leg, ServoParams y PCA9685Servo del código original sin modificarlos.

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

#include "hardware.h" // Leg, ServoParams (canales) y pwm del código real

static constexpr uint8_t PCA_ADDR = 0x40;

struct TestLeg
{
    const char *name;
    bool enabled;
    uint8_t ch[3]; // haa, hfe, kfe
};

// ===== PIERNAS A PROBAR: pon true/false =====
TestLeg LEGS[4] = {
    {"lf", 1, {ServoParams::lf_haa_channel, ServoParams::lf_hfe_channel, ServoParams::lf_kfe_channel}},
    {"rf", 1,  {ServoParams::rf_haa_channel, ServoParams::rf_hfe_channel, ServoParams::rf_kfe_channel}},
    {"lh", 1, {ServoParams::lh_haa_channel, ServoParams::lh_hfe_channel, ServoParams::lh_kfe_channel}},
    {"rh", 1, {ServoParams::rh_haa_channel, ServoParams::rh_hfe_channel, ServoParams::rh_kfe_channel}},
};

// SEQ = true  -> secuencial: prueba las piernas activas una a una
// SEQ = false -> todas las piernas activas se mueven a la vez (ojo: más consumo de corriente)
static constexpr bool SEQ = false;

// Pon false para solo ir a 0,0,0 sin recorrer el rango de cada articulación
static constexpr bool SWEEP = true;

// Rango dinámico de cada articulación [rad]. Sacado de ../../control/loquro_lab:
//   loquro.xml -> shoulder/knee/femur range="0 1.57" (las 12 articulaciones)
//   scripts LoQuRo_*.py -> usan hfe/kfe entre 0.0 y 1.5608 en el robot real
// Centro = 0.7854 (pose QUAD_INIT_CTRL). Ajusta aquí si choca con algo.
static constexpr float Q_MIN[3] = {0.0f, 0.0f, 0.0f}; // haa, hfe, kfe
static constexpr float Q_MAX[3] = {1.57f, 1.57f, 1.57f};
static constexpr float Q_MID = 0.7854f;
static constexpr float SPEED_RADS = 0.5f; // velocidad del barrido [rad/s], lento para no pedir picos de corriente
static constexpr uint32_t STEP_MS = 20;
static constexpr uint32_t HOLD_MS = 2000;

static bool pca_ok = false;
static float q[3] = {0.0f, 0.0f, 0.0f}; // última posición mandada (igual para todas las piernas activas)

Leg legs[4] = {
    Leg(&pwm, LEGS[0].ch[0], LEGS[0].ch[1], LEGS[0].ch[2]),
    Leg(&pwm, LEGS[1].ch[0], LEGS[1].ch[1], LEGS[1].ch[2]),
    Leg(&pwm, LEGS[2].ch[0], LEGS[2].ch[1], LEGS[2].ch[2]),
    Leg(&pwm, LEGS[3].ch[0], LEGS[3].ch[1], LEGS[3].ch[2]),
};
static bool active[4] = {false, false, false, false}; // piernas que se mueven en este momento
static char active_names[16] = "";                     // p.ej. "lf rh", para los mensajes

static void setActive(int only) // only = índice de una pierna, o -1 para todas las habilitadas
{
    active_names[0] = '\0';
    for (int i = 0; i < 4; i++)
    {
        active[i] = LEGS[i].enabled && (only < 0 || only == i);
        if (active[i])
        {
            strcat(active_names, LEGS[i].name);
            strcat(active_names, " ");
        }
    }
}

static void writeActive(float a, float b, float c)
{
    for (int i = 0; i < 4; i++)
        if (active[i])
            legs[i].write(a, b, c);
}

// Imprime por serie todas las direcciones I2C que responden en un bus (diagnóstico)
static void scan(TwoWire &bus, const char *name)
{
    Serial.printf("\n>> Escaneo I2C %s:", name);
    int n = 0;
    for (uint8_t addr = 8; addr < 120; addr++)
    {
        bus.beginTransmission(addr);
        if (bus.endTransmission() == 0)
        {
            Serial.printf(" 0x%02X", addr);
            n++;
        }
    }
    Serial.printf(n ? "\n" : " NINGUN dispositivo\n");
}

#define REQUIRE_PCA() \
    if (!pca_ok)      \
    TEST_IGNORE_MESSAGE("PCA9685 no detectada, no se mueve nada")

// Mueve la pierna en rampa (interpolación lineal) desde q hasta target a SPEED_RADS
static void rampTo(float t0, float t1, float t2)
{
    const float target[3] = {t0, t1, t2};
    float dist = 0.0f;
    for (int i = 0; i < 3; i++)
        dist = fmaxf(dist, fabsf(target[i] - q[i]));
    int steps = max(1, (int)(dist / SPEED_RADS * 1000.0f / STEP_MS));
    const float start[3] = {q[0], q[1], q[2]};
    for (int s = 1; s <= steps; s++)
    {
        float a = (float)s / steps;
        for (int i = 0; i < 3; i++)
            q[i] = start[i] + a * (target[i] - start[i]);
        writeActive(q[0], q[1], q[2]);
        delay(STEP_MS);
    }
}

// Barre una articulación: centro -> min -> max -> centro, con el resto en el centro
static void sweep(const char *name, int idx)
{
    REQUIRE_PCA();
    float t[3] = {Q_MID, Q_MID, Q_MID};
    rampTo(t[0], t[1], t[2]);

    Serial.printf(">> %s %s: %.2f -> %.2f (min)\n", active_names, name, Q_MID, Q_MIN[idx]);
    t[idx] = Q_MIN[idx];
    rampTo(t[0], t[1], t[2]);
    delay(HOLD_MS / 2);

    Serial.printf(">> %s %s: %.2f -> %.2f (max)\n", active_names, name, Q_MIN[idx], Q_MAX[idx]);
    t[idx] = Q_MAX[idx];
    rampTo(t[0], t[1], t[2]);
    delay(HOLD_MS / 2);

    Serial.printf(">> %s %s: vuelta al centro %.2f\n", active_names, name, Q_MID);
    t[idx] = Q_MID;
    rampTo(t[0], t[1], t[2]);
    delay(HOLD_MS / 2);
}

// ── Tests ─────────────────────────────────────────────────────

void test_pca9685_responde()
{
    Wire1.beginTransmission(PCA_ADDR);
    uint8_t rc = Wire1.endTransmission();
    pca_ok = (rc == 0);
    if (!pca_ok)
    {
        // Diagnóstico: ¿está en otro bus u otra dirección? (rc 2 = NACK, 5 = timeout/bus colgado)
        scan(Wire1, "Wire1 (SDA=17 SCL=18)");
        Wire.begin(8, 9);
        scan(Wire, "Wire  (SDA=8  SCL=9)");
    }
    TEST_ASSERT_EQUAL_MESSAGE(0, rc, "PCA9685 no responde en 0x40 (Wire1 SDA=17 SCL=18)");
}

void test_pierna_a_cero()
{
    REQUIRE_PCA();
    Serial.printf(">> %s -> write(0, 0, 0)\n", active_names);
    q[0] = q[1] = q[2] = 0.0f;
    writeActive(0.0f, 0.0f, 0.0f);
    delay(HOLD_MS);
    TEST_PASS(); // comprobación visual: la pierna debe estar en 0,0,0
}

// Orden de movimiento: haa -> hfe -> kfe
void test_rango_haa() { sweep("haa (cadera abd/add)", 0); }
void test_rango_hfe() { sweep("hfe (cadera flex/ext)", 1); }
void test_rango_kfe() { sweep("kfe (rodilla)", 2); }

// Ejecuta 0,0,0 + barridos sobre una pierna (only = índice) o todas las habilitadas (only = -1)
static void runLegTests(int only)
{
    setActive(only);
    Serial.printf("\n===== Piernas: %s=====\n", active_names);
    for (int i = 0; i < 4; i++)
        if (active[i])
            Serial.printf("      %s canales haa=%d hfe=%d kfe=%d\n", LEGS[i].name, LEGS[i].ch[0], LEGS[i].ch[1], LEGS[i].ch[2]);

    RUN_TEST(test_pierna_a_cero);
    if (SWEEP)
    {
        RUN_TEST(test_rango_haa);
        RUN_TEST(test_rango_hfe);
        RUN_TEST(test_rango_kfe);
    }
    if (pca_ok)
        rampTo(Q_MID, Q_MID, Q_MID); // deja las piernas en el centro al terminar
}

// ── Entrypoint ────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(2000);

    Wire1.end();
    Wire1.begin(17, 18);
    Wire1.setClock(400000);

    UNITY_BEGIN();
    RUN_TEST(test_pca9685_responde);

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PCA9685Cfg::freq);
    delay(10);

    if (SEQ)
    {
        for (int i = 0; i < 4; i++)
            if (LEGS[i].enabled)
                runLegTests(i);
    }
    else
    {
        runLegTests(-1); // todas las habilitadas a la vez
    }
    UNITY_END();
}

void loop() {}
