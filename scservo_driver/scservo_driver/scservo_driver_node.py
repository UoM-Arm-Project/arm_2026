import rclpy
from rclpy.node import Node
from scservo_sdk import PortHandler
from poker_interfaces.msg import ServoCommand

class SCServoDriver(Node):
    def __init__(self):
        super().__init__('scservo_driver')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('servo_ids', [1,2,3,4,5,6])

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.ids: list[int] = self.get_parameter('servo_ids').value

        portHandler = PortHandler(self.port)
        if not portHandler.openPort():
            self.get_logger().error(f"Failed to open port {self.port}")

        # NOTE: this closes and reopens the port, leaving as is as it follows
        # from the IMU example for scservo_sdk 
        if not portHandler.setBaudRate(self.baud):
            self.get_logger().error(f"Failed to set baud rate as {self.baud}")

        self.get_logger().info(f"Opened port {self.port} at {self.baud}")

        self.sub_ = self.create_subscription(
            ServoCommand, '/servo_cmd', self._cmd_cb, 10)

        self.pub_feedback_ = self.create_publisher(
            MotorFeedback, '/motor_feedback', 10)

        self.timer_ = self.create_timer(0.05, self._feedback_loop)

    # stump for servocommand subscription
    def _cmd_cb():
        return 0

    # stump for timer callback
    def _feedback_loop():
        return 0

def main(args=None):
    rclpy.init(args=args)
    node = SCServoDriver()
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