import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from bulldozer_interfaces.msg import MotorCommand

class SelectorNode(Node):
    def __init__(self):
        super().__init__('selector_node')
        self.get_logger().info("Selector Node Started")
        #STATE
        self.auto_enabled = False
        self.teleop_cmd = MotorCommand()
        self.auto_cmd = MotorCommand()

        #SUBSCRIBERS
        self.auto_enable_sub = self.create_subscription(
            Bool,
            '/auto_enable',
            self.auto_enable_callback,
            10
        )
        self.teleop_sub = self.create_subscription(
            MotorCommand,
            '/teleop_command',
            self.teleop_callback,
            10
        )
        self.auto_sub = self.create_subscription(
            MotorCommand,
            '/auto_command',
            self.auto_callback,
            10
        )

        #PUBLISHER
        self.motor_pub = self.create_publisher(
            MotorCommand,
            '/motor_command',
            10
        )

        #TIMER
        self.timer = self.create_timer(
            0.05,
            self.publish_command
        )
        
    #CALLBACK
    def auto_enable_callback(self, msg):
        self.auto_enabled = msg.data
    def teleop_callback(self, msg):
        self.teleop_cmd = msg
    def auto_callback(self, msg):
        self.auto_cmd = msg

    #SELECT COMMAND
    def publish_command(self):
        if self.auto_enabled:
            self.motor_pub.publish(
                self.auto_cmd
            )
        else:
            self.motor_pub.publish(
                self.teleop_cmd
            )

def main(args=None):
    rclpy.init(args=args)
    node = SelectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()