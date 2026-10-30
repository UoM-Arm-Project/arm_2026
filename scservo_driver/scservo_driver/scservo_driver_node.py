import rclpy
from rclpy.node import Node
from scservo_sdk import PortHandler
from scservo_sdk.sms_sts import sms_sts as SmsStsPacketHandler
from poker_interfaces.msg import ServoCommand, MotorFeedback

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
        # initialise packet handler
        self.packetHandler = SmsStsPacketHandler(PortHandler)

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

        if len(msg.ids) == 0:
            return

        for id, pos, speed, acceleration in \
            zip(msg.ids, msg.position, msg.speed, msg.acceleration):
            self.packetHandler.SyncWritePosEx(id, pos, speed, acceleration)

    # stump for timer callback
    def _feedback_loop(self):
        # Try to lock; if busy writing, skip this feedback cycle to prioritize control smoothness
        if self._lock.is_locked():
            return

        msg = MotorFeedback()
        msg.stamp = self.get_clock().now()
        msg.feedback_valid = True

        for id in self.ids:
            # read ONLY speed and position to save bus time
            pos = self.packetHandler.ReadPos(id)
            spd = self.packetHandler.ReadSpeed(id)

            if pos == -1:
                msg.feedback_valid = False

            msg.servo_ids.append(id)
            msg.positions.append(pos)
            msg.speeds.append(spd)

            # fill others with 0 to save bandwidth
            msg.loads.append(0)
            msg.voltages.append(0)
            msg.temperatures.append(0)

        self.pub_feedback_.publish(msg)
            

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