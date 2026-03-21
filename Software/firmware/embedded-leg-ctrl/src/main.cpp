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

  // QUEUES
  Queues::joint_states_queue = xQueueCreate(1, 4 * 3 * sizeof(float));
  Queues::time_stamp_queue = xQueueCreate(1, sizeof(double));
  Queues::imu_data_queue = xQueueCreate(1, sizeof(IMUdata));

  // IMU
  Serial.println("SYSTEM: Initializing IMU");
  imu.begin();
  imu.calibrate();

  imu.beginMag();
  imu.calibrateMag();

  // servos
  Serial.println("SYSTEM: Initializing PCA9685 Servos");
  Wire1.begin(17, 18);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PCA9685Cfg::freq); // 200Hz configurado desde el namespace
  delay(10);

  beginNetwork();

  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LED_PIN, HIGH);

  // threads declarations
  xTaskCreatePinnedToCore(watchdogTask, "watchdog", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(writeServosTask, "servos", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(readIMUTask, "imu_read", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sendStatusTask, "send_status", 4096, NULL, 2, NULL, 1); // core 1: latency min/avg/max/mdev = 4.512/7.733/22.248/2.542 ms
                                                                                  // core 0: latency min/avg/max/mdev = 4.872/9.381/62.122/4.254 ms
}

void loop()
{
  float joint_states_debug[4][3];

  // //---DEBUGGING---
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
}
#endif