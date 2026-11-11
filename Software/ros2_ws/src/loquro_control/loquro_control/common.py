"""Shared definitions of the LoQuRo nodes."""
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# order of the 12 positions of /joint_commands, the firmware uses this order (it ignores the names)
JOINT_NAMES = [
    'lf_haa', 'lf_hfe', 'lf_kfe',
    'rf_haa', 'rf_hfe', 'rf_kfe',
    'lh_haa', 'lh_hfe', 'lh_kfe',
    'rh_haa', 'rh_hfe', 'rh_kfe',
]
LEGS = ['lf', 'rf', 'lh', 'rh']

# joint commands: only the newest one matters, an old command must never delay a new one
COMMAND_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class ReadyMonitor:
    """Follows /robot/ready. The robot publishes it at 2 Hz: no message for `timeout` s means not reachable."""

    def __init__(self, node, timeout=1.5):
        from std_msgs.msg import Bool
        self._node = node
        self._timeout = timeout
        self._ready = False
        self._stamp = None
        node.create_subscription(Bool, 'robot/ready', self._callback, 10)

    def _callback(self, msg):
        self._ready = msg.data
        self._stamp = self._node.get_clock().now()

    @property
    def ready(self):
        if self._stamp is None:
            return False
        age = (self._node.get_clock().now() - self._stamp).nanoseconds * 1e-9
        return self._ready and age < self._timeout
