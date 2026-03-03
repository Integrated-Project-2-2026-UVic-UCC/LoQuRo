#include <zenoh-pico.h>
#include <ucdr/microcdr.h>

#include "config.h"
#include "hardware.h"
#include "mailbox.h"
#include "utilities.h"
#include "tasks.h"

#if Z_FEATURE_PUBLICATION == 1

z_owned_session_t s;      // session object
z_owned_subscriber_t sub; // subsciber object

// leg motors
uint8_t raw_buffer[512]; // buffer of 512 bytes to recieve raw data

// TODO: function to deserialize float32Multiarray etc
bool deserializeJointStates(ucdrBuffer *ub)
{ // TODO: Calculo latencia
  // Header stamped
  uint32_t sec, nanosec;
  ucdr_deserialize_uint32_t(ub, &sec);
  ucdr_deserialize_uint32_t(ub, &nanosec);

  double timestamp = (double)sec + (double)nanosec / 1e9; // seconds + nanoseconds in seconds, epoch time

  xQueueOverwrite(Queues::time_stamp_queue, &timestamp); // enviamos el timestamp a la tarea principal para calcular la latencia

  { // Frame id, by the moment we do not need it will be destroyed
    char frame_id[32];
    ucdr_deserialize_string(ub, frame_id, sizeof(frame_id));
  }

  { // all created vbles will be destroyed after this block
    uint32_t names_count;
    ucdr_deserialize_uint32_t(ub, &names_count); // number of names in the buffer
    char name_buf[32];
    for (uint32_t i = 0; i < names_count; i++) // read number of names and discard
      ucdr_deserialize_string(ub, name_buf, sizeof(name_buf));
  }
  uint32_t pos_count; // size of position array
  ucdr_deserialize_uint32_t(ub, &pos_count);

  if (pos_count < 12)
    return false; // we expect 12 positions, else the comunication failed or is not valid

  // CRÍTICO: Inicializar a 0.0 para evitar 'inf'
  double positions[12] = {0.0};

  // read all the postion array (we now we get doubles) (analized rawdata with AI)
  if (!ucdr_deserialize_array_double(ub, positions, 12))
    return false;

  float leg_buf[4][3]; // buffer for the joint_states queue
  for (int leg = 0; leg < 4; leg++)
  {
    leg_buf[leg][0] = (float)positions[leg * 3 + 0];
    leg_buf[leg][1] = (float)positions[leg * 3 + 1];
    leg_buf[leg][2] = (float)positions[leg * 3 + 2];
  }
  xQueueOverwrite(Queues::joint_states_queue, leg_buf);
  return true;
}

// zenoh callback to receive data
void data_handler(z_loaned_sample_t *sample, void *arg)
{
  const z_loaned_bytes_t *payload = z_sample_payload(sample);
  z_bytes_reader_t reader = z_bytes_get_reader(payload);
  size_t len = z_bytes_reader_remaining(&reader);

  if (len < 32 || len > sizeof(raw_buffer))
  {
    Serial.println(len); // the len is dynamic so if the names changes we cannot expect a fixed size
    return;              // only accept messages larger than 32 bytes (else there is no valid data)
  }
  z_bytes_reader_read(&reader, raw_buffer, len);

  ucdrBuffer ub;
  ucdr_init_buffer(&ub, raw_buffer + 4, len - 4); // pass first 4 bytes, header of message

  if (!deserializeJointStates(&ub))
    return;

  //---DEBUGGING---
  // for (int i = 0; i < len / sizeof(uint8_t); ++i)
  // {
  //   Serial.print(((uint8_t *)raw_buffer)[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
}
// TODO: WiFI manager implementation to run the robot everywere
// TASKS

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Queues::joint_states_queue = xQueueCreate(1, 4 * 3 * sizeof(float)); // init buffer to recive joint positions
  Queues::time_stamp_queue = xQueueCreate(1, sizeof(double));          // init buffer to recive timestamp for latency calculation
  // Queues::encoder_data_queue = xQueueCreate(1, );
  // Queues::imu_data_queue = xQueueCreate(1, );

  Serial.begin(115200);

  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
  }
  Serial.println("OK");

  // NTP clock sincronization por epoch time
  Serial.print("Sincronizando reloj NTP...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  while (time(nullptr) < 100000)
  { // wait to get a valid time
    delay(500);
    Serial.print(".");
  }

  // Initialize Zenoh Session and other parameters
  z_owned_config_t config;
  z_config_default(&config);
  zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
  if (strcmp(LOCATOR, "") != 0)
  {
    if (strcmp(MODE, "client") == 0)
    {
      zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, LOCATOR);
    }
    else
    {
      zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_LISTEN_KEY, LOCATOR);
    }
  }

  // Open Zenoh session
  Serial.print("Opening Zenoh Session...");
  if (z_open(&s, z_config_move(&config), NULL) < 0)
  {
    Serial.println("Unable to open session!");
    while (1)
    {
      ;
    }
  }
  Serial.println("OK");

  // Start read and lease tasks for zenoh-pico
  if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 || zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0)
  {
    Serial.println("Unable to start read and lease tasks\n");
    z_session_drop(z_session_move(&s));
    while (1)
    {
      ;
    }
  }
  // Declare Zenoh subscriber
  Serial.print("Declaring Subscriber on ");
  Serial.print(KEYEXPR);
  Serial.println(" ...");
  z_owned_closure_sample_t callback;
  z_closure_sample(&callback, data_handler, NULL, NULL);
  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
  if (z_declare_subscriber(z_session_loan(&s), &sub, z_view_keyexpr_loan(&ke), z_closure_sample_move(&callback),
                           NULL) < 0)
  {
    Serial.println("Unable to declare subscriber.");
    while (1)
    {
      ;
    }
  }
  Serial.println("OK");
  Serial.println("Zenoh setup finished!");

  digitalWrite(LED_PIN, HIGH);

  xTaskCreatePinnedToCore(
      watchdogTask,
      "watchdog_task",
      4096,
      NULL,
      1,
      NULL,
      0 // wifi connection
  );
  xTaskCreatePinnedToCore(
      writeServosTask,
      "write_servos_task",
      4096,
      NULL,
      3, // High priority
      NULL,
      1 // other core, 0 is for wifi
  );
}

void loop()
{
  float joint_states_debug[4][3];

  // LATENCY CALCULATION
  double timestamp_ros = 0.0;
  static double latency = 0.0;
  if (xQueueReceive(Queues::time_stamp_queue, &timestamp_ros, 0) == pdTRUE)
  {
    double current_time = getPreciseTime();
    latency = current_time - timestamp_ros;
  }

  // 3. DEBUGGER (Solo imprime 1 vez por segundo para no asfixiar al ESP32)
  static unsigned long last_print = 0;
  if (millis() - last_print >= 1000)
  {
    memcpy(joint_states_debug, &joint_states, 4 * 3 * sizeof(float));
    last_print = millis();
    Serial.println("\n--- [DEBUG] ESTADO DEL ROBOT ---");
    for (int leg = 0; leg < 4; leg++)
    {

      Serial.printf("Leg %d: HAA=%.4f | HFE=%.4f | KFE=%.4f rad\n",
                    leg, joint_states_debug[leg][0], joint_states_debug[leg][1], joint_states_debug[leg][2]);
    }
    Serial.printf("Latency: %.2f ms\n", latency * 1000.0);
  }
}

#endif
