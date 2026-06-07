## First-Time Setup

When powering on the robot for the first time, it will create a Wi-Fi access point called **LoQuRo**.

1. Turn on the robot.
2. Using a phone, tablet, or computer, connect to the **LoQuRo** Wi-Fi network.
3. A captive portal should open automatically. If it does not, reconnect to the network and wait a few seconds.
4. Enter:

   * Your Wi-Fi SSID
   * Your Wi-Fi password
   * The IP address of the device running ROS2 (Raspberry Pi, laptop, PC, etc.)
5. Save the configuration.

The ESP32 will automatically reboot and connect to the configured network.

Once connected, the robot can be started normally.

---

## Robot Startup and IMU Calibration

Every time the robot is powered on, the IMU must complete its calibration process.

When the **green LED turns on**:

1. Place the robot on a stable surface.
2. Keep it completely still for approximately **5 seconds**.

   * Gyroscope calibration is performed during this period.
   * Accelerometer calibration is also completed during this period.
3. Afterward, slowly move the robot through as many orientations as possible:

   * Rotate it around all axes.
   * Point it in different directions.
   * Avoid sudden movements.

This process allows the magnetometer to calibrate correctly.

Once calibration is finished, the robot will automatically move to its initial pose.

> [!IMPORTANT] A device running the ROS2 bridge must be connected to the same Wi-Fi network as the robot. Without the bridge running, communication between ROS2 and the ESP32 will not be established.

---

## ROS2 Bridge

The bridge is the core communication component between ROS2 (Fast DDS) and the ESP32 (Zenoh-Pico).

Start the bridge before launching any control or visualization nodes:

```bash
zenoh-bridge-ros2dds
```

The bridge must remain running during robot operation.

### Checking the Robot IP

The bridge output can be used to verify that the robot is connected and to identify the IP address assigned by the network.

---

## IMU Orientation Estimation (Madgwick Filter)

The robot publishes raw IMU and magnetometer measurements. To obtain a filtered orientation estimate, run the Madgwick filter:

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
  -r imu/data_raw:=/imu/data_raw \
  -r imu/mag:=/imu/mag \
  -p use_mag:=true
```

In another terminal:

```bash
rviz2
```

Inside RViz:

1. Set the fixed frame to:

   * `imu_link`, or
   * `base_link`
2. Click:

   * Add
   * By Topic
   * Select `imu/data`

The filtered orientation should now be visible.

> [!NOTE] Future versions will integrate the Madgwick filter directly into the ROS2 launch file so that it starts automatically.

---

## Control

After the bridge and IMU pipeline are running, control nodes can be launched.

Depending on the application, you can:

* Run the teleoperation node and manually control the robot.
* Launch autonomous control nodes.
* Execute test scripts such as:

```bash
python3 LoQuRo_xxxx.py
```

Ensure the bridge is active before running any control software.

---

## Testing

### Joint Command Test (Python)

This script publishes a fixed joint configuration and can be used to verify that the communication pipeline is working correctly.

The robot should move to a standing pose if commands are received successfully.

```bash
python3 -c "
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class P(Node):
    def __init__(self):
        super().__init__('p')
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_timer(0.02, self.cb)

    def cb(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.name = ['lf_haa','lf_hfe','lf_kfe',
                  'rf_haa','rf_hfe','rf_kfe',
                  'lh_haa','lh_hfe','lh_kfe',
                  'rh_haa','rh_hfe','rh_kfe']
        m.position = [0.7, 0.8, 0.8] * 4
        self.pub.publish(m)

rclpy.init()
rclpy.spin(P())
"
```

---

## Debug and Development Utilities

The firmware includes several debugging utilities that can be enabled directly in the source code when diagnosing hardware, communication, or control issues.

These tools are intended for development and troubleshooting purposes only.

### Encoder Monitoring

Prints encoder measurements for all joints through the serial monitor.

Useful for:

- Verifying encoder communication.
- Checking measured joint angles.
- Diagnosing calibration issues.

```cpp
static unsigned long last_print = 0;
static float enc_debug[12] = {0};

if (millis() - last_print >= 100)
{
    last_print = millis();

    if (xQueuePeek(Queues::encoder_data_queue, &enc_debug, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        Serial.println("\n--- [DEBUG] ENCODERS ---");

        const char *leg_names[4] = {"LF", "RF", "LH", "RH"};

        for (int leg = 0; leg < 4; leg++)
        {
            float haa = enc_debug[leg * 3 + 0];
            float hfe = enc_debug[leg * 3 + 1];
            float kfe = enc_debug[leg * 3 + 2];

            Serial.printf(
                "Leg %s: HAA=%.2f | HFE=%.2f | KFE=%.2f deg\n",
                leg_names[leg],
                haa,
                hfe,
                kfe);
        }
    }
}
```

### I2C Device Scanner

Scans both I2C buses and reports all detected devices.

Useful for:

- IMU troubleshooting.
- Encoder diagnostics.
- Hardware integration verification.

```cpp
byte error, address;
int nDevices;

nDevices = 0;
Serial.println("=== Scanning Wire (SDA=8, SCL=9) ===");

for (address = 1; address < 127; address++)
{
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
        Serial.print("Device found at 0x");
        if (address < 16)
            Serial.print("0");
        Serial.println(address, HEX);
        nDevices++;
    }
}

Serial.printf("%d device(s) found.\n", nDevices);

nDevices = 0;
Serial.println("=== Scanning Wire1 (SDA=17, SCL=18) ===");

for (address = 1; address < 127; address++)
{
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
        Serial.print("Device found at 0x");
        if (address < 16)
            Serial.print("0");
        Serial.println(address, HEX);
        nDevices++;
    }
}

Serial.printf("%d device(s) found.\n", nDevices);
```

### Joint State Monitoring

Prints the internal joint states processed by the controller.

Useful for:

- Verifying ROS command reception.
- Debugging kinematic calculations.
- Checking state estimation.

```cpp
static unsigned long last_print = 0;

if (millis() - last_print >= 1000)
{
    memcpy(joint_states_debug, &joint_states, 4 * 3 * sizeof(float));
    last_print = millis();

    Serial.println("\n--- [DEBUG] ROBOT STATE ---");

    for (int leg = 0; leg < 4; leg++)
    {
        Serial.printf(
            "Leg %d: HAA=%.4f | HFE=%.4f | KFE=%.4f rad\n",
            leg,
            joint_states_debug[leg][0],
            joint_states_debug[leg][1],
            joint_states_debug[leg][2]);
    }
}
```

### Servo Disable Mode

Disables all leg motion while keeping the rest of the firmware running.

Useful for:

- Sensor debugging.
- Communication testing.
- Safe development.

```cpp
leg_lf.write(0.0, 0.0, 0.0);
leg_rf.write(0.0, 0.0, 0.0);
leg_lh.write(0.0, 0.0, 0.0);
leg_rh.write(0.0, 0.0, 0.0);
```
