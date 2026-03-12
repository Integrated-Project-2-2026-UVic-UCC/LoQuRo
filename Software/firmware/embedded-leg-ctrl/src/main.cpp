#include "config.h"
#include "zenoh.h"
#include "hardware.h"
#include "mailbox.h"
#include "tasks.h"
#include "utilities.h"

#if Z_FEATURE_PUBLICATION == 1

// TODO: WiFI manager implementation to run the robot everywere
// TASKS

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
  }
  Serial.println("OK");

  initZenoh(); // init the zenoh config

  Serial.println(WiFi.localIP());

  digitalWrite(LED_PIN, HIGH);

  //////////////// Aditional configs /////////////////////////

  // queues declarations
  Queues::joint_states_queue = xQueueCreate(
      1, 4 * 3 * sizeof(float)); // init buffer to recive joint positions
  Queues::time_stamp_queue = xQueueCreate(
      1,
      sizeof(
          double)); // init buffer to recive timestamp for latency calculation
  // Queues::encoder_data_queue = xQueueCreate(1, );
  // Queues::imu_data_queue = xQueueCreate(1, );

  // threads declarations
  xTaskCreatePinnedToCore(watchdogTask, "watchdog_task", 4096, NULL, 1, NULL,
                          0 // wifi connection
  );
  xTaskCreatePinnedToCore(writeServosTask, "write_servos_task", 4096, NULL,
                          3, // High priority
                          NULL,
                          1 // other core, 0 is for wifi
  );
}

void loop()
{
  float joint_states_debug[4][3];

  // 3. DEBUGGER (Solo imprime 1 vez por segundo para no asfixiar al ESP32) (static to not be destoyed after exiting the loop function)
  static unsigned long last_print = 0;
  if (millis() - last_print >= 1000)
  {
    memcpy(joint_states_debug, &joint_states, 4 * 3 * sizeof(float));
    last_print = millis();
    Serial.println("\n--- [DEBUG] ESTADO DEL ROBOT ---");
    for (int leg = 0; leg < 4; leg++)
    {
      Serial.printf("Leg %d: HAA=%.4f | HFE=%.4f | KFE=%.4f rad\n", leg,
                    joint_states_debug[leg][0], joint_states_debug[leg][1],
                    joint_states_debug[leg][2]);
    }
  }
}

#endif
