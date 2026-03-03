### 1. Setup
Run this in a terminal to setup the bridge between ROS2 (Fast DDS) and ESP32 (Zenoh-pico).
```bash
zenoh-bridge-ros2dds*/ // open terminal to enable bridge
```
### 2. Testing
Code to test if the messages are being received correctly: 
<!--code to send jointstates message via python, fopr testing-->
```bash
python3 -c "
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
class P(Node):
    def __init__(self):
        super().__init__('p')
        self.a = 0.0
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_timer(0.1, self.cb)
    def cb(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.name = ['lf_haa','lf_hfe','lf_kfe','rf_haa','rf_hfe','rf_kfe','lh_haa','lh_hfe','lh_kfe','rh_haa','rh_hfe','rh_kfe']
        m.position = [self.a, self.a, self.a] * 4
        self.pub.publish(m)
        self.a+=0.1

rclpy.init()
rclpy.spin(P())
"
```
### Ideas descartadas
#### 1. Data handling

**handler:**
```cpp
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
```
Punteros:
```cpp
float *temp_leg_status[i][j] = &leg_status[i][j] // variable temporal para mover los servos sin que se sobreescriba durante la ejecucion
```
Memcpy:
```cpp
memcpy(leg_status, temp_leg_status, 4 * 3 * sizeof(float)); // copiamos el status a una variable temporal para mover los servos sin que se sobreescriba durante la ejecucion
```
**Mutex:**
```cpp
// threading (bona practica: cada vegada que accedim a una variable que canvia fem semafor, per evitar que es sobreescrigui mentre la llegim o escrivim)
// SemaphoreHandle_t data_mutex; // mutex para proteger el acceso a leg_status
//SNAPSHOT
xMutex = xSemaphoreCreateMutex();
xSemaphoreTake(xMutex, portMAX_DELAY);
// copiar leg_status a una variable temporal para mover los servos sin que se sobreescriba durante la ejecucion
memcpy(temp_leg_status, leg_status, 4 * 3 * sizeof(float));
xSemaphoreGive(xMutex); // liberamos el mutex despues de actualizar los servos
```
**DEADBAND:**
To know if the values changed significantly
```cpp
#define DEADBAND (3.0f * 3.14159265f / 2.0f) * 0.001f // threshold to consider a change in the leg status
```
