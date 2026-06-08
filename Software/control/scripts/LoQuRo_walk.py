import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Walk(Node):
    def __init__(self):
        super().__init__('walk')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(JointState, '/joint_commands', qos)

        self.lfd = [0.7, 1.5608, 0.8196]  # Left front down
        self.lfu = [0.7, 0.3502, 1.4249]  # Left front up
        self.rfd = [0.7, 0.0392, 0.7804]  # Right front down
        self.rfu = [0.7, 1.2498, 0.1751]  # Right front up
        self.lrd = [0.7, 1.5608, 0.8196]  # Left rear down
        self.lru = [0.7, 0.3502, 1.4249]  # Left rear up
        self.rrd = [0.7, 0.0392, 0.7804]  # Right rear down
        self.rru = [0.7, 1.2498, 0.1751]  # Right rear up

        # ─────────────────────────────────────────
        # RUTINA: lista de poses que se ejecutan en orden
        # Cada pose es [lf_haa, lf_hfe, lf_kfe,
        #               rf_haa, rf_hfe, rf_kfe,
        #               lh_haa, lh_hfe, lh_kfe,
        #               rh_haa, rh_hfe, rh_kfe]
        # ─────────────────────────────────────────
        self.routine = [
            # POSE 0 — de pie estable
            self.lfd + self. rfd + self.lrd + self.rrd,

            # POSE 1 — tuneame
            self.lfu + self. rfu + self.lru + self.rru,
        ]
        # ─────────────────────────────────────────

        self.step_size = 0.02   # velocidad de interpolación (rad por tick)
        self.tolerance = 0.015  # margen para considerar pose alcanzada

        self.current_state = 0
        self.target_position = self.routine[self.current_state]
        self.current_position = self.target_position[:]

        self.create_timer(0.05, self.update_position)  # 20 Hz

    def interpolate(self, current, target):
        new_pos = []
        for c, t in zip(current, target):
            if abs(t - c) < self.step_size:
                new_pos.append(t)
            else:
                new_pos.append(c + self.step_size * (1 if t > c else -1))
        return new_pos

    def reached_target(self, current, target):
        return all(abs(c - t) < self.tolerance for c, t in zip(current, target))

    def update_position(self):
        self.current_position = self.interpolate(self.current_position, self.target_position)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'lf_haa', 'lf_hfe', 'lf_kfe',
            'rf_haa', 'rf_hfe', 'rf_kfe',
            'lh_haa', 'lh_hfe', 'lh_kfe',
            'rh_haa', 'rh_hfe', 'rh_kfe',
        ]
        msg.position = self.current_position
        self.pub.publish(msg)

        if self.reached_target(self.current_position, self.target_position):
            self.current_state = (self.current_state + 1) % len(self.routine)
            self.target_position = self.routine[self.current_state]

def main(args=None):
    rclpy.init(args=args)
    node = Walk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()