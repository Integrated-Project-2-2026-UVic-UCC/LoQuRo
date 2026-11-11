"""Plays a routine of poses (routines/<name>.yaml) on the robot.

1. Waits until the robot is ready. The robot is then at its initial pose (INIT_POSE in the firmware).
2. Moves slowly to the first pose of the routine and waits `settle_time` seconds.
3. Goes through the sequence moving every joint at most at `speed` rad/s.

If the robot stops being ready (communication lost, calibration) the routine pauses and continues from the same
position. If it is not ready for more than `restart_after` seconds it is assumed that the robot restarted (it is
back at its initial pose) and the routine starts again.
"""
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from loquro_control.common import COMMAND_QOS, JOINT_NAMES, LEGS, ReadyMonitor


def load_routine(name):
    path = Path(name)
    if not path.suffix:
        path = Path(get_package_share_directory('loquro_control')) / 'routines' / f'{name}.yaml'
    data = yaml.safe_load(path.read_text())

    leg_poses = data.get('leg_poses', {})
    poses = {}
    for pose_name, pose in data['poses'].items():
        if isinstance(pose, dict):  # {lf: [haa, hfe, kfe] or leg pose name, ...}
            pose = sum((list(leg_poses.get(pose[leg], pose[leg])) for leg in LEGS), [])
        if len(pose) != 12:
            raise ValueError(f'pose "{pose_name}" must have 12 positions')
        poses[pose_name] = [float(p) for p in pose]

    default_speed = float(data.get('speed', 0.4))
    steps = [{'pose': poses[step['pose']],
              'name': step['pose'],
              'speed': float(step.get('speed', default_speed)),
              'hold': float(step.get('hold', 0.0))} for step in data['sequence']]
    return steps, bool(data.get('loop', False)), default_speed


class RoutinePlayer(Node):

    def __init__(self):
        super().__init__('routine_player')
        self.declare_parameter('routine', 'squats')
        # must match INIT_POSE in the firmware (include/config.h)
        self.declare_parameter('initial_pose', [0.7, 0.8, 0.8] * 4)
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('restart_after', 5.0)

        name = self.get_parameter('routine').value
        self.steps, self.loop, self.approach_speed = load_routine(name)
        self.get_logger().info(f'Routine "{name}": {len(self.steps)} steps, loop={self.loop}')

        self.ready = ReadyMonitor(self)
        self.pub = self.create_publisher(JointState, 'joint_commands', COMMAND_QOS)
        self.dt = 1.0 / self.get_parameter('rate').value
        self.not_ready_time = 0.0
        self.restart()
        self.create_timer(self.dt, self.update)

    def restart(self):
        self.current = list(self.get_parameter('initial_pose').value)
        self.index = 0
        self.speed = self.approach_speed
        self.wait = 0.0
        self.waiting = False
        self.finished = False

    def update(self):
        if not self.ready.ready:
            self.not_ready_time += self.dt
            if self.not_ready_time > self.get_parameter('restart_after').value and self.index > 0:
                self.get_logger().warn('Robot not ready for too long, the routine starts again')
                self.restart()
            return
        self.not_ready_time = 0.0
        if self.finished:
            return

        step = self.steps[self.index]
        if not self.waiting:
            max_delta = self.speed * self.dt
            self.current = [c + max(-max_delta, min(max_delta, t - c)) for c, t in zip(self.current, step['pose'])]
            if self.current == step['pose']:
                # the first pose is reached slowly from the initial pose: wait until the robot settles
                self.wait = self.get_parameter('settle_time').value if self.index == 0 else step['hold']
                self.waiting = True
        else:
            self.wait -= self.dt
            if self.wait <= 0.0:
                self.next_step()
        self.publish()

    def next_step(self):
        self.waiting = False
        self.index += 1
        if self.index >= len(self.steps):
            if not self.loop:
                self.finished = True
                self.get_logger().info('Routine finished')
                return
            self.index = 0
        self.speed = self.steps[self.index]['speed']

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.current
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RoutinePlayer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
