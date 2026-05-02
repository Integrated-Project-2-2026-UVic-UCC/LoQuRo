#include "config.h"
#include "hardware.h"
#include "mailbox.h"
#include "tasks.h"
#include "utilities.h"

#if Z_FEATURE_PUBLICATION == 1

// DONE: WiFI manager implementation to run the robot everywere

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  Wire.end();
  Wire1.end();

  Wire.begin(8, 9);
  Wire.setClock(400000); // bus freq

  Wire1.begin(17, 18);
  Wire1.setClock(400000);

  // QUEUES
  Queues::joint_states_queue = xQueueCreate(1, 4 * 3 * sizeof(float));
  Queues::time_stamp_queue = xQueueCreate(1, sizeof(double));
  Queues::imu_data_queue = xQueueCreate(1, sizeof(IMUdata));
  Queues::encoder_data_queue = xQueueCreate(1, 4 * 3 * sizeof(float));

  beginNetwork();

  // ENCODERS - init BEFORE network (I2C must be ready)
  Serial.println("SYSTEM: Initializing Encoders");
  joints.begin();

  Serial.println("SYSTEM: Initializing PCA9685 Servos");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PCA9685Cfg::freq); // 200Hz configurado desde el namespace
  delay(10);

  // IMU
  // imu.begin();
  // imu.calibrate();
  // imu.beginMag();
  // imu.calibrateMag();

  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LED_PIN, HIGH);

  // threads declarations
  xTaskCreatePinnedToCore(watchdogTask, "watchdog", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(writeServosTask, "servos", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(readIMUTask, "imu_read", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(readEncodersTask, "encoders_read", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sendStatusTask, "send_status", 4096, NULL, 2, NULL, 1); // core 1: latency min/avg/max/mdev = 4.512/7.733/22.248/2.542 ms
  // core 0: latency min/avg/max/mdev = 4.872/9.381/62.122/4.254 ms

  // pal jan: rutina homing a zero
  // leg_lf.write(0.0, 0.0, 0.0); // for debug, to not move the robot
  // leg_rf.write(0.0, 0.0, 0.0);
  // leg_lh.write(0.0, 0.0, 0.0);
  // leg_rh.write(0.0, 0.0, 0.0);
}

void loop()
{
  // //---DEBUGGING: I2C Scanner---
  // for (uint8_t addr = 0; addr <= 127; addr++)
  // {
  //   Wire.beginTransmission(addr);
  //   if (!Wire.endTransmission())
  //   {
  //     Serial.print("Encontrado I2C 0x");
  //     Serial.println(addr, HEX);
  //   }
  // }
  // Serial.println("Scanning Multiplexor...");
  // for (uint8_t t = 0; t < 8; t++)
  // {
  //   Wire.beginTransmission(0x70);
  //   Wire.write(1 << t);
  //   Wire.endTransmission();

  //   Serial.print("  Escaneando salida ");
  //   Serial.println(t);

  //   for (uint8_t addr = 0; addr <= 127; addr++)
  //   {
  //     if (addr == 0x70)
  //       continue;

  //     Wire.beginTransmission(addr);
  //     if (!Wire.endTransmission())
  //     {
  //       Serial.print("  - Encontrado I2C 0x");
  //       Serial.println(addr, HEX);
  //     }
  //   }
  // }
  // Serial.println("Finalizado");
  // float joint_states_debug[4][3];

  // // //---DEBUGGING JOINTS---
  // static unsigned long last_print = 0;
  // if (millis() - last_print >= 1000)
  // {
  //   memcpy(joint_states_debug, &joint_states, 4 * 3 * sizeof(float));
  //   last_print = millis();
  //   Serial.println("\n--- [DEBUG] ESTADO DEL ROBOT ---");
  //   for (int leg = 0; leg < 4; leg++)
  //   {
  //     Serial.printf("Leg %d: HAA=%.4f | HFE=%.4f | KFE=%.4f rad\n", leg,
  //                   joint_states_debug[leg][0], joint_states_debug[leg][1],
  //                   joint_states_debug[leg][2]);
  //   }
  // }

  // ─── Debbuging: SQUATS TEST ───────────────────────────────────────────────

  // static const float POS_DOWN[4][3] = {
  //     {0.7f, 1.5608f, 0.8196f}, // LF down
  //     {0.7f, 0.0392f, 0.7804f}, // RF down
  //     {0.7f, 1.5608f, 0.8196f}, // LR down
  //     {0.7f, 0.0392f, 0.7804f}, // RR down
  // };
  // static const float POS_UP[4][3] = {
  //     {0.7f, 0.3502f, 1.4249f}, // LF up
  //     {0.7f, 1.2498f, 0.1751f}, // RF up
  //     {0.7f, 0.3502f, 1.4249f}, // LR up
  //     {0.7f, 1.2498f, 0.1751f}, // RR up
  // };

  // static const float STEP = 0.01f;  // rad por tick
  // static const uint32_t PERIOD = 50; // ms entre ticks (20 Hz)
  // // ───────────────────────────────────────────────────────────────

  // static float cur[4][3];
  // static float tgt[4][3];
  // static uint32_t t0 = 0;
  // static bool going_up = true;
  // static bool init_done = false;

  // if (!init_done)
  // {
  //   memcpy(cur, POS_DOWN, sizeof(cur));
  //   memcpy(tgt, POS_UP, sizeof(tgt));
  //   t0 = millis();
  //   init_done = true;
  // }

  // uint32_t now = millis();
  // if (now - t0 < PERIOD)
  //   return;
  // t0 = now;

  // // Interpola y detecta si todas las articulaciones llegaron
  // bool done = true;
  // for (int l = 0; l < 4; l++)
  // {
  //   for (int j = 0; j < 3; j++)
  //   {
  //     float diff = tgt[l][j] - cur[l][j];
  //     if (diff > STEP)
  //     {
  //       cur[l][j] += STEP;
  //       done = false;
  //     }
  //     else if (diff < -STEP)
  //     {
  //       cur[l][j] -= STEP;
  //       done = false;
  //     }
  //     else
  //     {
  //       cur[l][j] = tgt[l][j];
  //     }
  //   }
  // }

  // // Escribe todos los servos de una vez
  // leg_lf.write(cur[0][0], cur[0][1], cur[0][2]);
  // leg_rf.write(cur[1][0], cur[1][1], cur[1][2]);
  // leg_lh.write(cur[2][0], cur[2][1], cur[2][2]);
  // leg_rh.write(cur[3][0], cur[3][1], cur[3][2]);

  // // Flip al llegar al objetivo
  // if (done)
  // {
  //   going_up = !going_up;
  //   memcpy(tgt, going_up ? POS_UP : POS_DOWN, sizeof(tgt));
  // }

  // one articulation test, it doesnt work better. HIGH resolution
  // static const float POS_DOWN = 1.5608f; // Posición "bajada" para la articulación
  // static const float POS_UP = 0.3502f;   // Posición "subida" para la articulación
  // static const float STEP = 0.01f;       // Paso pequeño para alta resolución
  // static const uint32_t PERIOD = 50;     // 50 ms entre actualizaciones (~20 Hz)

  // static float current_position = POS_DOWN; // Posición actual de la articulación
  // static float target_position = POS_UP;    // Posición objetivo
  // static uint32_t last_update = 0;          // Marca de tiempo para control de frecuencia
  // static bool going_up = true;              // Dirección del movimiento

  // // Control de frecuencia (~100 Hz)
  // uint32_t now = millis();
  // if (now - last_update < PERIOD)
  //   return;
  // last_update = now;

  // // Interpolar hacia la posición objetivo
  // float diff = target_position - current_position;
  // if (abs(diff) > STEP)
  // {
  //   current_position += STEP * (diff > 0 ? 1 : -1); // Incremento o decremento
  // }
  // else
  // {
  //   current_position = target_position; // Alcanzó la posición objetivo
  // }

  // // Escribir la posición actual en la articulación (ejemplo: LF HFE)
  // leg_lf.write(0.0, current_position, 0.0); // Solo mueve la articulación HFE

  // // Cambiar la dirección al alcanzar el objetivo
  // if (current_position == target_position)
  // {
  //   going_up = !going_up;
  //   target_position = going_up ? POS_UP : POS_DOWN;
  // }
}
#endif