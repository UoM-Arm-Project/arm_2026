import rclpy
from rclpy.node import Node
from scservo_sdk import PortHandler
from poker_interfaces.msg import ServoCommand

from threading import Lock

class SCServoDriver(Node):
    def __init__(self):
        super().__init__('scservo_driver')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('servo_ids', [1,2,3,4,5,6])

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.ids: list[int] = self.get_parameter('servo_ids').value

        # NOTE: Check if this works, if not revert to previous commit
        # driver source code setBaudRate() calls setupPort which opens the port

        portHandler = PortHandler(self.port)
        # opens port
        if portHandler.setBaudRate(self.baud):
            self.get_logger().info(f"Opened port {self.port} at {self.baud}")
        else:
            self.get_logger().error(f"Failed to open port {self.port} at {self.baud}")
        
        self.sub_ = self.create_subscription(
            ServoCommand, '/servo_cmd', self._cmd_cb, 10)

        self.pub_feedback_ = self.create_publisher(
            MotorFeedback, '/motor_feedback', 10)

        self.timer_ = self.create_timer(0.05, self._feedback_loop)

        self._lock = Lock()

    # stump for servocommand subscription
    def _cmd_cb(self, msg: ServoCommand):
        self._lock.acquire()

        count = len(msg.ids)
        if count == 0:
            return
        

        return 0

    # stump for timer callback
    def _feedback_loop(self):
        # Try to lock; if busy writing, skip this feedback cycle to prioritize control smoothness
        if self._lock.is_locked():
            return

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