"""ROS 2 driver node for the STS3215 servo chain.

Subscribes to /servo_velocity_cmd (signed steps/s per joint), forwards them to
the bus in wheel mode via one sync-write packet, and publishes /motor_feedback
at 20 Hz. A watchdog zeroes all speeds if commands stop arriving.
"""

import rclpy
from rclpy.node import Node

from poker_interfaces.msg import MotorFeedback, ServoVelocityCommand
from .sts3215 import STS3215Bus


class STS3215DriverNode(Node):
    def __init__(self):
        super().__init__('sts3215_driver')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('cmd_timeout_ms', 500)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.servo_ids = list(self.get_parameter('servo_ids').value)
        self.cmd_timeout = self.get_parameter('cmd_timeout_ms').value / 1000.0

        self.bus = STS3215Bus(port, baud, self.servo_ids)
        if not self.bus.connect():
            self.get_logger().error(f'Failed to open port {port}')
        else:
            self.get_logger().info(f'Port {port} opened at {baud}')
            self.bus.set_wheel_mode()

        self._last_cmd_time = self.get_clock().now()
        self._stopped = True  # start with speeds already zero

        self.sub_cmd = self.create_subscription(
            ServoVelocityCommand, '/servo_velocity_cmd',
            self.cmd_callback, 10)

        self.pub_feedback = self.create_publisher(
            MotorFeedback, '/motor_feedback', 10)

        # 20 Hz feedback - safe limit for 6 servos @ 1 Mbps
        self.feedback_timer = self.create_timer(0.05, self.feedback_loop)
        # Watchdog: wheel mode holds the last speed, so stale commands are
        # dangerous. Zero the bus if the controller goes quiet.
        self.watchdog_timer = self.create_timer(0.1, self.watchdog)

    def cmd_callback(self, msg):
        speeds = {}
        for servo_id, speed in zip(msg.ids, msg.speeds):
            speeds[int(servo_id)] = int(speed)
        if not speeds:
            return
        if not self.bus.port.is_open:
            return
        self.bus.set_speeds(speeds)
        self._last_cmd_time = self.get_clock().now()
        self._stopped = all(s == 0 for s in speeds.values())

    def feedback_loop(self):
        if not self.bus.port.is_open:
            return

        msg = MotorFeedback()
        msg.stamp = self.get_clock().now().to_msg()
        msg.feedback_valid = True

        for servo_id, (pos, spd) in self.bus.read_all().items():
            if pos == -1:
                msg.feedback_valid = False
            msg.servo_ids.append(servo_id)
            msg.positions.append(pos)
            msg.speeds.append(spd)
            msg.loads.append(0)
            msg.voltages.append(0)
            msg.temperatures.append(0)

        self.pub_feedback.publish(msg)

    def watchdog(self):
        if self._stopped or not self.bus.port.is_open:
            return
        elapsed = (self.get_clock().now()
                   - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.get_logger().warn(
                'Command timeout - zeroing servo speeds')
            self.bus.stop_all()
            self._stopped = True

    def destroy_node(self):
        if self.bus.port.is_open:
            self.bus.stop_all()
            self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STS3215DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
