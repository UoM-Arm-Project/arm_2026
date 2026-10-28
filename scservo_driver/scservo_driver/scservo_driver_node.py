import rclpy
from rclpy.node import Node

class SCServoDriver(Node):
    def __init__(self):
        super().__init__('scservo_driver')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', '1000000')
        self.declare_parameter('servo_ids', [1,2,3,4,5,6])

        self.port = self.get_parameter('port')
        self.baud = self.get_parameter('baud')
        self.ids: list[int] = self.get_parameter('servo_ids').value

        # open ports

        # subscribe to /servo_cmd

        # publish to /motor_feedback

        # create timer

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