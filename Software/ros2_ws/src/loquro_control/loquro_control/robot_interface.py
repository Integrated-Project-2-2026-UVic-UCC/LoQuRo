"""Interface with the robot firmware (through the zenoh bridge).

- Keeps every topic of the robot declared in ROS, so the bridge always creates its routes: the IMU topics exist
  even with nothing else running, and the requests to the robot are never lost.
- Publishes the PC clock in /robot/time (1 Hz): the robot uses it to timestamp its messages (no NTP needed).
- Action /calibrate_imu (loquro_interfaces/action/CalibrateImu): recalibrates the IMU of the robot.
- Logs when the robot connects / disconnects.
"""
import threading

import rclpy
from builtin_interfaces.msg import Time
from loquro_interfaces.action import CalibrateImu
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Empty, UInt8

from loquro_control.common import COMMAND_QOS, ReadyMonitor

# firmware CalibrationResult (include/tasks.h)
CALIBRATION_RESULTS = {
    0: (True, 'IMU calibrated (accelerometer, gyroscope and magnetometer)'),
    1: (False, 'The robot has no IMU (accelerometer / gyroscope not answering)'),
    2: (True, 'Accelerometer and gyroscope calibrated, magnetometer not available'),
    3: (False, 'Accelerometer and gyroscope calibrated, magnetometer rejected: rotate the robot more'),
    4: (False, 'Not calibrated: the robot is receiving joint commands, stop the routine / policy first'),
}


class RobotInterface(Node):

    def __init__(self):
        super().__init__('robot_interface')
        self.declare_parameter('calibration_timeout', 60.0)
        group = ReentrantCallbackGroup()

        self.ready = ReadyMonitor(self)
        self.create_subscription(Imu, 'imu/data_raw', lambda _: None, qos_profile_sensor_data)
        self.create_subscription(MagneticField, 'imu/mag', lambda _: None, qos_profile_sensor_data)

        self.time_pub = self.create_publisher(Time, 'robot/time', COMMAND_QOS)
        self.create_timer(1.0, self.publish_time)

        self.calib_pub = self.create_publisher(Empty, 'imu/calibrate', 10)
        self.create_subscription(UInt8, 'imu/calibration_result', self.on_calibration_result, 10,
                                 callback_group=group)
        self.calib_done = threading.Event()
        self.calib_result = None
        ActionServer(self, CalibrateImu, 'calibrate_imu', self.calibrate,
                     goal_callback=lambda _: GoalResponse.ACCEPT, callback_group=group)

        self.was_ready = False
        self.create_timer(0.5, self.log_connection)

    def publish_time(self):
        self.time_pub.publish(self.get_clock().now().to_msg())

    def log_connection(self):
        if self.ready.ready != self.was_ready:
            self.was_ready = self.ready.ready
            if self.was_ready:
                self.get_logger().info('Robot ready')
            else:
                self.get_logger().warn('Robot not ready (disconnected, booting or calibrating)')

    def on_calibration_result(self, msg):
        self.calib_result = msg.data
        self.calib_done.set()

    def calibrate(self, goal):
        result = CalibrateImu.Result()
        if not self.ready.ready:
            goal.abort()
            result.message = 'The robot is not ready'
            return result

        self.calib_done.clear()
        self.calib_pub.publish(Empty())
        feedback = CalibrateImu.Feedback()
        feedback.phase = 'Keep the robot still, then rotate it in all directions when the LED blinks fast'
        goal.publish_feedback(feedback)

        if not self.calib_done.wait(self.get_parameter('calibration_timeout').value):
            goal.abort()
            result.message = 'No answer from the robot'
            return result

        result.success, result.message = CALIBRATION_RESULTS.get(
            self.calib_result, (False, f'Unknown result {self.calib_result}'))
        goal.succeed() if result.success else goal.abort()
        return result


def main():
    rclpy.init()
    node = RobotInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
