import rclpy, math, time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class P(Node):
    def __init__(self):
        super().__init__('p')
        # Configuración de QoS para garantizar la entrega de mensajes
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Garantiza la entrega
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(JointState, '/joint_commands', qos)
        
        # Definimos las posiciones de bajada y subida para cada pata
        self.lfd = [0.7, 1.5608, 0.8196]  # Left front down
        self.lfu = [0.7, 0.3502, 1.4249]  # Left front up
        self.rfd = [0.7, 0.0392, 0.7804]  # Right front down
        self.rfu = [0.7, 1.2498, 0.1751]  # Right front up
        self.lrd = [0.7, 1.5608, 0.8196]  # Left rear down
        self.lru = [0.7, 0.3502, 1.4249]  # Left rear up
        self.rrd = [0.7, 0.0392, 0.7804]  # Right rear down
        self.rru = [0.7, 1.2498, 0.1751]  # Right rear up

        # Posición de la rodilla contraída y oscilación del abductor
        self.lf_knee_contracted = [0.7, 0.0, 0.0]  # Left front knee contracted
        self.lf_abductor_oscillate = [0.5, 0.0, 0.0]  # Abductor at 0.5
        self.lf_abductor_oscillate_high = [0.9, 0.0, 0.0]  # Abductor at 0.9

        # Estados de la rutina
        self.states = [
            self.lfd + self.rfd + self.lrd + self.rrd,  # Todas las patas abajo
            self.lfu + self.rfu + self.lrd + self.rrd,  # Subir solo las patas delanteras
            self.lf_knee_contracted + self.rfu + self.lrd + self.rrd,  # Contraer rodilla izquierda
            self.lf_abductor_oscillate + self.rfu + self.lrd + self.rrd,  # Oscilación abductor (inicio)
            self.lf_abductor_oscillate_high + self.rfu + self.lrd + self.rrd,  # Oscilación abductor (arriba)
            self.lf_abductor_oscillate + self.rfu + self.lrd + self.rrd,  # Oscilación abductor (abajo)
            self.lf_abductor_oscillate_high + self.rfu + self.lrd + self.rrd,  # Oscilación abductor (arriba)
            self.lfu + self.rfu + self.lrd + self.rrd,  # Estirar patas delanteras
            self.lfd + self.rfd + self.lrd + self.rrd   # Todas las patas abajo
        ]
        self.current_state = 0
        self.target_state = self.states[self.current_state]
        self.current_position = self.target_state[:]
        self.default_step_size = 0.02  # Movimientos normales más rápidos
        self.fast_step_size = 0.05    # Movimientos rápidos aún más rápidos
        self.create_timer(0.03, self.cb)  # Llamar cada 30 ms (~33 Hz)

    def interpolate(self, current, target, step_size):
        """Interpolar gradualmente entre la posición actual y la posición objetivo."""
        new_position = []
        for c, t in zip(current, target):
            if abs(t - c) < step_size:
                new_position.append(t)
            else:
                new_position.append(c + step_size * (1 if t > c else -1))
        return new_position

    def cb(self):
        # Determinar el tamaño del paso según el estado actual
        if self.current_state in [2, 3, 4, 5, 6, 7]:  # Estados de contracción y oscilación
            step_size = self.fast_step_size
        else:
            step_size = self.default_step_size

        # Interpolar hacia el estado objetivo
        self.current_position = self.interpolate(self.current_position, self.target_state, step_size)

        # Publicar la posición actual interpolada
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ['lf_haa','lf_hfe','lf_kfe','rf_haa','rf_hfe','rf_kfe','lh_haa','lh_hfe','lh_kfe','rh_haa','rh_hfe','rh_kfe']
        m.position = self.current_position
        self.pub.publish(m)

        # Verificar si hemos alcanzado el estado objetivo
        if self.current_position == self.target_state:
            # Cambiar al siguiente estado
            self.current_state = (self.current_state + 1) % len(self.states)
            self.target_state = self.states[self.current_state]

rclpy.init()
try:
    rclpy.spin(P())
except KeyboardInterrupt:
    pass