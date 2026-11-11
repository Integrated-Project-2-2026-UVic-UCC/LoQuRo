// Herramienta para MEDIR en el robot real la banda valida de (hfe - kfe) de una pierna.
//
// La rodilla se mueve con una biela: el mecanismo solo cierra mientras (hfe - kfe) esta dentro de una banda, y en
// los bordes esta la singularidad (tensor alineado con la palanca de la tibia). Este programa fija hfe en varios
// valores y mueve kfe MUY despacio hacia cada lado: tu lo paras con una tecla ANTES de llegar al punto muerto.
//
// No es un test automatico, es interactivo. Uso (con el robot levantado, las piernas en el aire):
//   pio test -e esp32-s3-devkitc-1 -f test_leg_limits --without-testing    (compila y sube)
//   pio device monitor                                                      (para escribir las teclas)
//
// MUEVE MOTORES. Ten la mano cerca de la alimentacion.

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "hardware.h" // Leg, ServoParams y pwm del codigo real

static constexpr float HAA = 0.7f;                               // abductor fijo durante la medida
static constexpr float HFE_VALUES[] = {0.4f, 0.6f, 0.8f, 1.0f, 1.2f}; // valores de hfe donde se mide
static constexpr float Q_MIN = 0.0f, Q_MAX = 1.57f;              // rango de cada servo
static constexpr float SWEEP_SPEED = 0.08f;                      // rad/s buscando el limite (muy lento)
static constexpr float MOVE_SPEED = 0.4f;                        // rad/s para ir a cada posicion
static constexpr float MARGIN = 0.10f;                           // rad que se quitan al limite medido
static constexpr uint32_t STEP_MS = 20;

struct LegDef
{
    const char *name;
    bool left;
    uint8_t ch[3];
};
static const LegDef LEGS[4] = {
    {"lf", true, {ServoParams::lf_haa_channel, ServoParams::lf_hfe_channel, ServoParams::lf_kfe_channel}},
    {"rf", false, {ServoParams::rf_haa_channel, ServoParams::rf_hfe_channel, ServoParams::rf_kfe_channel}},
    {"lh", true, {ServoParams::lh_haa_channel, ServoParams::lh_hfe_channel, ServoParams::lh_kfe_channel}},
    {"rh", false, {ServoParams::rh_haa_channel, ServoParams::rh_hfe_channel, ServoParams::rh_kfe_channel}},
};

static Leg *leg = nullptr;
static float q_hfe = INIT_POSE[1], q_kfe = INIT_POSE[2]; // ultima posicion mandada

static void flushInput()
{
    while (Serial.available())
        Serial.read();
}

// mueve hfe y kfe en rampa. Con stop_on_key devuelve true si se pulso una tecla antes de llegar ('q' = abortar)
static bool rampTo(float hfe, float kfe, float speed, bool stop_on_key, char *key = nullptr)
{
    float dist = fmaxf(fabsf(hfe - q_hfe), fabsf(kfe - q_kfe));
    int steps = max(1, (int)(dist / speed * 1000.0f / STEP_MS));
    float hfe0 = q_hfe, kfe0 = q_kfe;
    for (int s = 1; s <= steps; s++)
    {
        if (stop_on_key && Serial.available())
        {
            char c = Serial.read();
            if (key)
                *key = c;
            flushInput();
            return true;
        }
        float a = (float)s / steps;
        q_hfe = hfe0 + a * (hfe - hfe0);
        q_kfe = kfe0 + a * (kfe - kfe0);
        leg->write(HAA, q_hfe, q_kfe);
        delay(STEP_MS);
    }
    return false;
}

static String readLine()
{
    String line;
    for (;;)
    {
        while (!Serial.available())
            delay(10);
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (line.length())
                return line;
        }
        else
            line += c;
    }
}

// busca el limite moviendo kfe hacia kfe_end. Devuelve true y la diferencia (hfe - kfe) si se paro con una tecla
static bool findLimit(float hfe, float kfe_end, float *diff, bool *abort)
{
    Serial.printf("   kfe %.2f -> %.2f ... pulsa una tecla ANTES del punto muerto ('q' = abortar)\n", q_kfe, kfe_end);
    flushInput();
    char key = 0;
    bool stopped = rampTo(hfe, kfe_end, SWEEP_SPEED, true, &key);
    *abort = (key == 'q');
    *diff = q_hfe - q_kfe;
    if (stopped)
        Serial.printf("   LIMITE: kfe = %.3f   hfe - kfe = %+.3f\n", q_kfe, *diff);
    else
        Serial.printf("   sin limite: se llego al final del servo (hfe - kfe = %+.3f)\n", *diff);
    rampTo(hfe, hfe, MOVE_SPEED, false); // vuelve al centro de la banda (hfe = kfe)
    return stopped;
}

static void measure(const LegDef &def)
{
    Leg this_leg(&pwm, def.ch[0], def.ch[1], def.ch[2]);
    leg = &this_leg;
    q_hfe = INIT_POSE[1];
    q_kfe = INIT_POSE[2];
    leg->write(HAA, q_hfe, q_kfe);
    delay(1000);

    // limite mas restrictivo encontrado en cada lado
    float diff_min = -M_PI, diff_max = M_PI;
    bool found_min = false, found_max = false, abort = false;

    for (float hfe : HFE_VALUES)
    {
        Serial.printf("\n== %s  hfe = %.2f ==\n", def.name, hfe);
        rampTo(hfe, hfe, MOVE_SPEED, false);
        float diff;

        if (findLimit(hfe, Q_MAX, &diff, &abort) && !abort) // kfe sube: hfe - kfe negativo
        {
            diff_min = fmaxf(diff_min, diff);
            found_min = true;
        }
        if (abort)
            break;
        if (findLimit(hfe, Q_MIN, &diff, &abort) && !abort) // kfe baja: hfe - kfe positivo
        {
            diff_max = fminf(diff_max, diff);
            found_max = true;
        }
        if (abort)
            break;
    }

    rampTo(INIT_POSE[1], INIT_POSE[2], MOVE_SPEED, false);
    leg = nullptr;

    if (abort)
    {
        Serial.println("\nAbortado.");
        return;
    }

    // the right legs are mirrored (right = 1.6 - left): (hfe - kfe) changes sign. Results in the left convention
    float lo = diff_min, hi = diff_max;
    bool has_lo = found_min, has_hi = found_max;
    if (!def.left)
    {
        lo = -diff_max;
        hi = -diff_min;
        has_lo = found_max;
        has_hi = found_min;
    }
    Serial.printf("\n===== RESULTADO %s =====\n", def.name);
    Serial.println("Banda medida de (hfe - kfe), convencion pierna IZQUIERDA, sin margen:");
    Serial.printf("   min: %s   max: %s\n", has_lo ? String(lo, 2).c_str() : "sin limite", has_hi ? String(hi, 2).c_str() : "sin limite");
    Serial.printf("Con %.2f rad de margen, para include/config.h (DIFF_MIN / DIFF_MAX):\n", MARGIN);
    Serial.printf("   DIFF_MIN = %s   DIFF_MAX = %s\n", has_lo ? String(lo + MARGIN, 2).c_str() : "-", has_hi ? String(hi - MARGIN, 2).c_str() : "-");
    Serial.println("Referencia (rf y simulacion): min -0.94, max +0.91. Si sale mucho mas estrecho, se paro antes de tiempo.");
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Wire1.end();
    Wire1.begin(17, 18);
    Wire1.setClock(400000);

    Wire1.beginTransmission(0x40);
    if (Wire1.endTransmission() != 0)
    {
        for (;;)
        {
            Serial.println("PCA9685 no responde en 0x40, revisa el cableado");
            delay(2000);
        }
    }
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PCA9685Cfg::freq);
    delay(10);
}

void loop()
{
    Serial.println("\nMedida de limites de pierna. Robot levantado, piernas en el aire.");
    Serial.println("Escribe la pierna a medir (lf, rf, lh, rh) y pulsa Enter:");
    flushInput();
    String name = readLine();
    name.trim();
    for (const LegDef &def : LEGS)
    {
        if (name == def.name)
        {
            measure(def);
            return;
        }
    }
    Serial.printf("Pierna desconocida: '%s'\n", name.c_str());
}
