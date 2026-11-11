"""PLACEHOLDER: runs a locomotion policy (ONNX, trained in Software/control/loquro_lab) on the robot.

Not usable yet. By default it only runs the policy and logs the output, it does not move the robot
(parameter `publish`). Before publishing, solve the TODOs:

- TODO: the policy observes joint positions and velocities (encoders), the robot has none. Here the last commanded
  position is used as an estimation, it may not be enough.
- TODO: joint mapping. The simulation uses fl/fr/rl/rr legs and shoulder/knee/femur joints, the firmware
  lf/rf/lh/rh and haa/hfe/kfe. It is not checked which joint is which.
- TODO: in the real robot the right legs are mirrored (same pose: left 1.56, right 0.04), in the simulation all the
  joints use 0-1.57 with 0.785 at the default pose.
- TODO: the IMU axes must match the simulated sensor (imu_site in loquro.xml).
"""
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

from loquro_control.common import COMMAND_QOS, JOINT_NAMES, ReadyMonitor

ACTION_SCALE = 0.25                         # env_cfgs.py: actions["joint_pos"].scale
DEFAULT_JOINT_POS = np.full(12, 0.7854)     # quad_bot_constants.py: QUAD_INIT_CTRL (approximation)


class PolicyPlayer(Node):

    def __init__(self):
        super().__init__('policy_player')
        self.declare_parameter('model', '')
        self.declare_parameter('publish', False)
        self.declare_parameter('rate', 50.0)  # training control rate (play.py CONTROL_DT)

        import onnxruntime as ort
        model = Path(self.get_parameter('model').value)
        if not model.is_file():
            raise FileNotFoundError(f'model not found: "{model}" (parameter model)')
        self.session = ort.InferenceSession(str(model), providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name

        self.ready = ReadyMonitor(self)
        self.imu = None
        self.command = np.zeros(3, dtype=np.float32)
        self.targets = DEFAULT_JOINT_POS.copy()
        self.last_targets = self.targets.copy()
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.pub = self.create_publisher(JointState, 'joint_commands', COMMAND_QOS)
        self.dt = 1.0 / self.get_parameter('rate').value
        self.create_timer(self.dt, self.step)
        self.get_logger().warn('PLACEHOLDER policy player, see the TODOs in policy_player.py')

    def on_imu(self, msg):
        self.imu = msg

    def on_cmd_vel(self, msg):
        self.command[:] = [msg.linear.x, msg.linear.y, msg.angular.z]

    def observation(self):
        a, w = self.imu.linear_acceleration, self.imu.angular_velocity
        joint_pos = self.targets - DEFAULT_JOINT_POS             # TODO: estimation, no encoders
        joint_vel = (self.targets - self.last_targets) / self.dt  # TODO: estimation, no encoders
        accel = np.clip([a.x, a.y, a.z], -30.0, 30.0) * 0.1
        gyro = np.clip([w.x, w.y, w.z], -10.0, 10.0)
        obs = np.concatenate([joint_pos, joint_vel, accel, gyro, self.command]).astype(np.float32)
        return obs.reshape(1, -1)

    def step(self):
        if self.imu is None or not self.ready.ready:
            return
        action = self.session.run(None, {self.input_name: self.observation()})[0][0]
        self.last_targets = self.targets
        self.targets = action * ACTION_SCALE + DEFAULT_JOINT_POS  # TODO: joint mapping and mirrored legs

        if not self.get_parameter('publish').value:
            self.get_logger().info(f'targets: {np.round(self.targets, 3).tolist()}', throttle_duration_sec=1.0)
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.targets.astype(float).tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PolicyPlayer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
