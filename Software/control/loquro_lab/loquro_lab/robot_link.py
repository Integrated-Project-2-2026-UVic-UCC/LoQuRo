"""Sends joint commands to the real robot without ROS, through the zenoh bridge of `robot.launch.py`.

The robot speaks zenoh (zenoh-pico) and the bridge is also a zenoh router: this module connects to it as one more
zenoh client and writes the same messages a ROS node would publish (sensor_msgs/JointState in CDR).
Needs the launch running (ros2 launch loquro_bringup robot.launch.py) in this PC or in `endpoint`.
"""
import struct
import time

import zenoh

JOINT_NAMES = [f"{leg}_{joint}" for leg in ("lf", "rf", "lh", "rh") for joint in ("haa", "hfe", "kfe")]


def joint_state_cdr(positions) -> bytes:
    """sensor_msgs/JointState serialized as ROS 2 does (CDR little endian)."""
    now = time.time()
    body = bytearray(struct.pack("<iI", int(now), int((now % 1) * 1e9)))

    def align(n):
        body.extend(b"\0" * (-len(body) % n))

    def string(text):
        align(4)
        data = text.encode() + b"\0"
        body.extend(struct.pack("<I", len(data)) + data)

    string("")  # frame_id
    align(4)
    body.extend(struct.pack("<I", len(JOINT_NAMES)))
    for name in JOINT_NAMES:
        string(name)
    align(4)
    body.extend(struct.pack("<I", len(positions)))
    align(8)
    body.extend(struct.pack(f"<{len(positions)}d", *positions))
    for _ in range(2):  # velocity, effort: empty
        align(4)
        body.extend(struct.pack("<I", 0))
    return b"\x00\x01\x00\x00" + bytes(body)


class RobotLink:

    def __init__(self, endpoint="tcp/127.0.0.1:7447"):
        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", f'["{endpoint}"]')
        self.session = zenoh.open(config)
        self._ready = False
        self._ready_time = 0.0
        self._sub = self.session.declare_subscriber("robot/ready", self._on_ready)
        self._pub = self.session.declare_publisher("joint_commands",
                                                   congestion_control=zenoh.CongestionControl.DROP,
                                                   priority=zenoh.Priority.REAL_TIME)

    def _on_ready(self, sample):
        payload = sample.payload.to_bytes()
        self._ready = len(payload) > 4 and payload[4] != 0  # std_msgs/Bool after the CDR header
        self._ready_time = time.monotonic()

    @property
    def ready(self):
        """The robot publishes robot/ready at 2 Hz: true while it accepts joint commands."""
        return self._ready and time.monotonic() - self._ready_time < 1.5

    def send(self, positions):
        """12 positions [rad], order lf rf lh rh x [haa, hfe, kfe]."""
        self._pub.put(joint_state_cdr([float(p) for p in positions]))

    def close(self):
        self.session.close()
