import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Squats(Node):
    def __init__(self):
        super().__init__('squats')
        # Configuración de QoS para garantizar la entrega de mensajes
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(JointState, '/joint_commands', qos)

        # Definimos las posiciones de bajada y subida para las patas
        self.down = [0.7, 1.5608, 0.8196,  # Left front down
                     0.7, 0.0392, 0.7804,  # Right front down
                     0.7, 1.5608, 0.8196,  # Left rear down
                     0.7, 0.0392, 0.7804]  # Right rear down

        self.up = [0.7, 0.3502, 1.4249,  # Left front up
                   0.7, 1.2498, 0.1751,  # Right front up
                   0.7, 0.3502, 1.4249,  # Left rear up
                   0.7, 1.2498, 0.1751]  # Right rear up

        # Configuración inicial
        self.current_position = self.down[:]  # Comienza en la posición "down"
        self.target_position = self.up  # Primera transición será hacia "up"
        self.step_size = 0.02  # Tamaño del paso para interpolación (ajusta para velocidad)
        self.create_timer(0.05, self.update_position)  # Llamar cada 50 ms (~20 Hz)

    def interpolate(self, current, target, step_size):
        """Interpolar gradualmente entre la posición actual y la posición objetivo."""
        new_position = []
        for c, t in zip(current, target):
            if abs(t - c) < step_size:
                new_position.append(t)
            else:
                new_position.append(c + step_size * (1 if t > c else -1))
        return new_position

    def update_position(self):
        """Actualizar la posición de las patas y publicar el mensaje."""
        # Interpolar hacia la posición objetivo
        self.current_position = self.interpolate(self.current_position, self.target_position, self.step_size)

        # Publicar la posición actual
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['lf_haa', 'lf_hfe', 'lf_kfe',
                    'rf_haa', 'rf_hfe', 'rf_kfe',
                    'lh_haa', 'lh_hfe', 'lh_kfe',
                    'rh_haa', 'rh_hfe', 'rh_kfe']
        msg.position = self.current_position
        self.pub.publish(msg)

        # Si alcanzamos la posición objetivo, alternar entre "up" y "down"
        if self.current_position == self.target_position:
            self.target_position = self.up if self.target_position == self.down else self.down

def main(args=None):
    rclpy.init(args=args)
    node = Squats()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    