#include "config.h"
#include "hardware.h"
#include "mailbox.h"
#include "tasks.h"
#include "utilities.h"

#ifndef PIO_UNIT_TESTING // the tests (test/ folder) have their own setup() and loop()

void setup()
{
  Serial.begin(115200);
  setState(STATE_ERROR); // red until the hardware is checked
  xTaskCreatePinnedToCore(statusLedTask, "status_led", 2048, NULL, 1, NULL, 0);

  Wire.end();
  Wire1.end();

  Wire.begin(8, 9);
  Wire.setClock(400000); // bus freq

  Wire1.begin(17, 18);
  Wire1.setClock(400000);

  // Servos
  Serial.println("SYSTEM: Initializing PCA9685 Servos");
  Wire1.beginTransmission(0x40);
  while (Wire1.endTransmission() != 0) // without servo driver the robot is not operative
  {
    Serial.println("SYSTEM: PCA9685 not found, check the wiring");
    delay(2000);
    Wire1.beginTransmission(0x40);
  }
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PCA9685Cfg::freq); // 100Hz from namespace
  delay(10);                        // debouncing

  // knee linkage band, the right legs are mirrored
  leg_lf.setCoupledLimit(DIFF_MIN, DIFF_MAX);
  leg_lh.setCoupledLimit(DIFF_MIN, DIFF_MAX);
  leg_rf.setCoupledLimit(-DIFF_MAX, -DIFF_MIN);
  leg_rh.setCoupledLimit(-DIFF_MAX, -DIFF_MIN);

  // IMU: if it does not answer the robot works without it (yellow led) and readIMUTask keeps retrying
  imu_ok = imu.begin();
  mag_ok = imu_ok && imu.beginMag();
  if (imu_ok)
  {
    if (imu.loadCalibration())
    {
      Serial.println("IMU: Saved calibration loaded");
      imu.printCalibration();
    }
    else
      Serial.println("IMU: No saved calibration, calibrate it with the /calibrate_imu action");
  }

  // QUEUES
  Queues::joint_states_queue = xQueueCreate(1, 4 * 3 * sizeof(float));
  Queues::imu_data_queue = xQueueCreate(1, sizeof(IMUdata));

  // WiFi + connection with the ROS2 bridge (blocks until connected)
  beginNetwork();

  // communication ready: initial pose, wait until the robot is at rest and calibrate the gyro bias (2 seconds)
  setState(STATE_CALIB_STILL);
  leg_lf.write(INIT_POSE[0], INIT_POSE[1], INIT_POSE[2]);
  leg_rf.write(INIT_POSE[0], INIT_POSE[1], INIT_POSE[2]);
  leg_lh.write(INIT_POSE[0], INIT_POSE[1], INIT_POSE[2]);
  leg_rh.write(INIT_POSE[0], INIT_POSE[1], INIT_POSE[2]);
  if (imu_ok)
    imu.calibrate(false);
  else
    delay(2000); // without imu, fixed time to reach the position

  // threads declarations
  xTaskCreatePinnedToCore(watchdogTask, "watchdog", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(writeServosTask, "servos", 4096, NULL, 3, NULL, 1);
  // the tasks that publish with zenoh need more stack (8 KB): a zenoh put over UDP overflowed 4 KB
  xTaskCreatePinnedToCore(readIMUTask, "imu_read", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sendStatusTask, "send_status", 8192, NULL, 2, NULL, 1); // core 1: latency min/avg/max/mdev = 4.512/7.733/22.248/2.542 ms
                                                                                  //  core 0: latency min/avg/max/mdev = 4.872/9.381/62.122/4.254 ms

  // ready: from now on joint commands are accepted
  setState(STATE_READY);
}

void loop()
{
}
#endif
