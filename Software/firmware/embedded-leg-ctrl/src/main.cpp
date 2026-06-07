#include "config.h"
#include "hardware.h"
#include "mailbox.h"
#include "tasks.h"
#include "utilities.h"

#if Z_FEATURE_PUBLICATION == 1

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

  // indicate network setup done
  digitalWrite(LED_PIN, HIGH);

  // Servos
  Serial.println("SYSTEM: Initializing PCA9685 Servos");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PCA9685Cfg::freq); // 200Hz from namespace
  delay(10);                        // debouncing

  // IMU
  imu.begin();
  imu.calibrate();
  imu.beginMag();
  imu.calibrateMag();

  // homing to rest
  leg_lf.write(0.7, 0.8, 0.8);
  leg_rf.write(0.7, 0.8, 0.8);
  leg_lh.write(0.7, 0.8, 0.8);
  leg_rh.write(0.7, 0.8, 0.8);
  //---DEBUGGING--- Pos 0
  // leg_lf.write(0.0, 0.0, 0.0);
  // leg_rf.write(0.0, 0.0, 0.0);
  // leg_lh.write(0.0, 0.0, 0.0);
  // leg_rh.write(0.0, 0.0, 0.0);

  // threads declarations
  xTaskCreatePinnedToCore(watchdogTask, "watchdog", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(writeServosTask, "servos", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(readIMUTask, "imu_read", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(readEncodersTask, "encoders_read", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sendStatusTask, "send_status", 4096, NULL, 2, NULL, 1); // core 1: latency min/avg/max/mdev = 4.512/7.733/22.248/2.542 ms
                                                                                  //  core 0: latency min/avg/max/mdev = 4.872/9.381/62.122/4.254 ms
}

void loop()
{
}
#endif