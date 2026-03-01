/*zenoh-bridge-ros2dds*/ // open terminal to enable bridge
/*
python3 -c "
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class P(Node):
    def __init__(self):
        super().__init__('p')
        pub = self.create_publisher(JointState, '/joint_commands', 10)
        def cb():
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.name = ['lf_haa','lf_hfe','lf_kfe','rf_haa','rf_hfe','rf_kfe','lh_haa','lh_hfe','lh_kfe','rh_haa','rh_hfe','rh_kfe']
            m.position = [0.0, 0.7, -1.4] * 4
            pub.publish(m)
        self.create_timer(0.1, cb)

rclpy.init()
rclpy.spin(P())
"
*/
// code to send jointstates message via python, fopr testing

#include <ServoCtrl.h>
#include <WiFi.h>
#include <zenoh-pico.h>
#include <ucdr/microcdr.h>
#include <WiFiManager.h>

#if Z_FEATURE_PUBLICATION == 1

// TODO: WiFI manager implementation to run the robot everywere

#define SSID "Red WiFI"
#define PASS "Password"

// Zenoh-specific parameters
#define MODE "client"
// #define LOCATOR "tcp/172.20.10.2:7447"
#define LOCATOR "tcp/192.168.0.36:7447" // If empty, scout

#define KEYEXPR "joint_commands" // ros topic

// to send data to the esp32
#define LED_PIN 2

const TickType_t whatchdog_delay = pdMS_TO_TICKS(500); // 2hz

z_owned_session_t s;      // session object
z_owned_subscriber_t sub; // subsciber object

namespace Queues
{
  QueueHandle_t joint_states_queue; // queue to send positions to the loop (movement task)
  QueueHandle_t time_stamp_queue;   // queue to send timestamp to the loop (latency calculation)
  QueueHandle_t encoder_data_queue; // queue to send encoder data to the send status task
  QueueHandle_t imu_data_queue;     // queue to send IMU data to the send status task
}

// leg motors
uint8_t raw_buffer[512];                                                                         // buffer of 512 bytes to recieve raw data
float joint_states[4][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}; // LF, RF, LH, RH - HAA, HFE, KFE. It will be changeing, cannot be static
#define DEADBAND (3.0f * 3.14159265f / 2.0f) * 0.001f                                            // threshold to consider a change in the leg status

struct Leg
{
  ServoCtrl haa; // hip abductor adductor
  ServoCtrl hfe; // hip flexor extensor
  ServoCtrl kfe; // knee flexor extensor

  Leg(int haa_pin, int hfe_pin, int kfe_pin, int haa_channel, int hfe_channel, int kfe_channel, int resolution)
      : haa(haa_pin, haa_channel, resolution), hfe(hfe_pin, hfe_channel, resolution), kfe(kfe_pin, kfe_channel, resolution)
  {
  }
  void setLimit(float haa_limit, float hfe_limit, float kfe_limit)
  {
    haa.setLimit(haa_limit);
    hfe.setLimit(hfe_limit);
    kfe.setLimit(kfe_limit);
  }
  void write(float haa_rads, float hfe_rads, float kfe_rads)
  {
    haa.write(haa_rads);
    hfe.write(hfe_rads);
    kfe.write(kfe_rads);
  }
  void writeDegrees(float haa_degs, float hfe_degs, float kfe_degs)
  {
    haa.writeDegrees(haa_degs);
    hfe.writeDegrees(hfe_degs);
    kfe.writeDegrees(kfe_degs);
  }
};

namespace ServoParams
{
  static constexpr int lf_hha_pin = 16;
  static constexpr int lf_hfe_pin = 17;
  static constexpr int lf_kfe_pin = 18;
  static constexpr int rf_hha_pin = 19;
  static constexpr int rf_hfe_pin = 21;
  static constexpr int rf_kfe_pin = 22;
  static constexpr int lh_hha_pin = 23;
  static constexpr int lh_hfe_pin = 25;
  static constexpr int lh_kfe_pin = 26;
  static constexpr int rh_hha_pin = 27;
  static constexpr int rh_hfe_pin = 32;
  static constexpr int rh_kfe_pin = 33;

  static constexpr int lf_haa_channel = 0;  // Canal LEDC (0-15)
  static constexpr int lf_hfe_channel = 1;  // Canal LEDC (0-15)
  static constexpr int lf_kfe_channel = 2;  // Canal LEDC (0-15)
  static constexpr int rf_haa_channel = 3;  // Canal LEDC (0-15)
  static constexpr int rf_hfe_channel = 4;  // Canal LEDC (0-15)
  static constexpr int rf_kfe_channel = 5;  // Canal LEDC (0-15)
  static constexpr int lh_haa_channel = 6;  // Canal LEDC (0-15)
  static constexpr int lh_hfe_channel = 7;  // Canal LEDC (0-15)
  static constexpr int lh_kfe_channel = 8;  // Canal LEDC (0-15)
  static constexpr int rh_haa_channel = 9;  // Canal LEDC (0-15)
  static constexpr int rh_hfe_channel = 10; // Canal LEDC (0-15)
  static constexpr int rh_kfe_channel = 11; // Canal LEDC (0-15)

  constexpr int resolution = 16; // Resolución (1-16 bits)
}

Leg leg_lf(ServoParams::lf_hha_pin, ServoParams::lf_hfe_pin, ServoParams::lf_kfe_pin, ServoParams::lf_haa_channel, ServoParams::lf_hfe_channel, ServoParams::lf_kfe_channel, ServoParams::resolution);
Leg leg_rf(ServoParams::rf_hha_pin, ServoParams::rf_hfe_pin, ServoParams::rf_kfe_pin, ServoParams::rf_haa_channel, ServoParams::rf_hfe_channel, ServoParams::rf_kfe_channel, ServoParams::resolution);
Leg leg_lh(ServoParams::lh_hha_pin, ServoParams::lh_hfe_pin, ServoParams::lh_kfe_pin, ServoParams::lh_haa_channel, ServoParams::lh_hfe_channel, ServoParams::lh_kfe_channel, ServoParams::resolution);
Leg leg_rh(ServoParams::rh_hha_pin, ServoParams::rh_hfe_pin, ServoParams::rh_kfe_pin, ServoParams::rh_haa_channel, ServoParams::rh_hfe_channel, ServoParams::rh_kfe_channel, ServoParams::resolution);

// TODO: function to deserialize jointstates, float32Multiarray etc

// UTILITY FUNCTIONS
double getPreciseTime() // standard function to get epoch time via NTP
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

bool deserializeJointStates(ucdrBuffer *ub)
{
  // Header stamped
  uint32_t sec, nanosec;
  ucdr_deserialize_uint32_t(ub, &sec);
  ucdr_deserialize_uint32_t(ub, &nanosec);

  double timestamp = (double)sec + (double)nanosec / 1e9; // seconds + nanoseconds in seconds, epoch time
  xQueueOverwrite(Queues::time_stamp_queue, &timestamp);  // enviamos el timestamp a la tarea principal para calcular la latencia

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

// TASKS
void watchdogTask(void *pvParameters)
{
  for (;;)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      digitalWrite(LED_PIN, LOW);
      WiFi.reconnect();                // TODO: It do not reconnects, maybe we need to do WiFi.begin() again?
      vTaskDelay(pdMS_TO_TICKS(3000)); // greater delay to give time to reconnect
    }
    else
      vTaskDelay(whatchdog_delay); // delay to not overload CPU
  }
}

void readEncodersTask(void *pvParameters)
{
  for (;;)
  {
    // TODO: read encoders and pin to core
    vTaskDelay(pdMS_TO_TICKS(10)); // 100hz
  }
}

void readIMUTask(void *pvParameters)
{
  for (;;)
  {
    // TODO: read IMU and pin to core
    vTaskDelay(pdMS_TO_TICKS(10)); // 100hz
  }
}

void sendStatusTask(void *pvParameters)
{
  for (;;)
  {
    // TODO: Send status (encoder data, imu data) to ROS2 through zenoh-pico and pin to core
    vTaskDelay(pdMS_TO_TICKS(10)); // ajustar frecuencia de envio de status
  }
}

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
}

void loop()
{
  if (xQueueReceive(Queues::joint_states_queue, &joint_states, pdMS_TO_TICKS(100)) == pdTRUE) // if there is data in the buffer
  {
    // MOVE SERVOS
    leg_lf.write(joint_states[0][0], joint_states[0][1], joint_states[0][2]);
    leg_rf.write(joint_states[1][0], joint_states[1][1], joint_states[1][2]);
    leg_lh.write(joint_states[2][0], joint_states[2][1], joint_states[2][2]);
    leg_rh.write(joint_states[3][0], joint_states[3][1], joint_states[3][2]);

    // LATENCY CALCULATION
    double timestamp_ros = 0.0;
    double latency = 0.0;
    if (xQueueReceive(Queues::time_stamp_queue, &timestamp_ros, 0) == pdTRUE)
    {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      double current_time = (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
      latency = current_time - timestamp_ros;
    }

    //   // 3. DEBUGGER (Solo imprime 1 vez por segundo para no asfixiar al ESP32)
    //   static unsigned long last_print = 0;
    //   if (millis() - last_print >= 1000)
    //   {
    //     last_print = millis();
    //     Serial.println("\n--- [DEBUG] ESTADO DEL ROBOT ---");
    //     for (int leg = 0; leg < 4; leg++)
    //     {
    //       Serial.printf("Leg %d: HAA=%.4f | HFE=%.4f | KFE=%.4f rad\n",
    //                     leg, joint_states[leg][0], joint_states[leg][1], joint_states[leg][2]);
    //     }
    //     Serial.printf("Latency: %.2f ms\n", latency * 1000.0);
    //   }
  }
}

#endif

/*
//Ideas descartadas:
  loop:
    memcpy(leg_status, temp_leg_status, 4 * 3 * sizeof(float)); // copiamos el status a una variable temporal para mover los servos sin que se sobreescriba durante la ejecucion
  handler:
    for (int j = 0; j < 4; ++j)
    {
      for (int i = 0; i < 3; ++i)
      {
        if (fabsf(last_leg_status[j][i] - leg_status[j][i]) > DEADBAND) // si hi ha una nova dada copiem tot i marxem
        {
          // static float last_leg_status[4][3] = {*leg_lf, *leg_lh, *leg_rf, *leg_rh};
          //  Copiamos cada pata a su fila correspondiente en el status
          memcpy(last_leg_status, leg_status, 4 * 3 * sizeof(float))
          return;
        }
      }
    }
    // threading (bona practica: cada vegada que accedim a una variable que canvia fem semafor, per evitar que es sobreescrigui mentre la llegim o escrivim)
    // SemaphoreHandle_t data_mutex; // mutex para proteger el acceso a leg_status
    //SNAPSHOT
    xMutex = xSemaphoreCreateMutex();
    xSemaphoreTake(xMutex, portMAX_DELAY);
    // copiar leg_status a una variable temporal para mover los servos sin que se sobreescriba durante la ejecucion
    memcpy(temp_leg_status, leg_status, 4 * 3 * sizeof(float));
    xSemaphoreGive(xMutex); // liberamos el mutex despues de actualizar los servos

    float *temp_leg_status[i][j] = &leg_status[i][j] // variable temporal para mover los servos sin que se sobreescriba durante la ejecucion

*/