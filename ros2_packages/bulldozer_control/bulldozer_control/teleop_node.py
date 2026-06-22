import rclpy
from rclpy.node import Node
from std_msgs import msg
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from bulldozer_interfaces.msg import MotorCommand
from std_msgs.msg import Float32

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.get_logger().info(
            "Teleop Node Started"
        )

        #INITIALIZATION
        self.servo_angle = 90
        self.distance = 999.0
        self.auto_enabled = False
        self.last_y_button = 0
        self.max_pwm = 100

        #PUBLISHER
        self.publisher = self.create_publisher(
            MotorCommand,
            '/teleop_command',
            10
        )
        self.auto_pub = self.create_publisher(
            Bool,
            '/auto_enable',
            10
        )
        #JOYSTICK SUBSCRIBER
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.distance_sub = self.create_subscription(
            Float32,
            '/distance',
            self.distance_callback,
            10
        )

    #JOYSTICK CALLBACK
    def joy_callback(self, msg):
        motor_msg = MotorCommand()
        #AUTO MODE TOGGLE
        y_button = msg.buttons[4]
        if y_button == 1 and self.last_y_button == 0:
            self.auto_enabled = (
                not self.auto_enabled
            )
            auto_msg = Bool()
            auto_msg.data = (
                self.auto_enabled
            )
            self.auto_pub.publish(
                auto_msg
            )
            self.get_logger().info(
                f"AUTO MODE: {self.auto_enabled}"
            )
        self.last_y_button = y_button
        #DRIVE CONTROL
        steering = msg.axes[0]
        rt = msg.axes[4]
        lt = msg.axes[5]
        rt = (1 - rt) / 2
        lt = (1 - lt) / 2
        throttle = rt - lt
        #ROTATE MODE
        if abs(throttle) < 0.00005:
            left = -steering
            right = steering
        #NORMAL DRIVE
        else:
            if throttle < 0:
                steering = -steering
            left = throttle - (steering * 0.7)
            right = throttle + (steering * 0.7)
        #PWM
        left_pwm = int(left * self.max_pwm)
        right_pwm = int(right * self.max_pwm)
        left_pwm = max(
            min(left_pwm, self.max_pwm),
            -self.max_pwm
        )
        right_pwm = max(
            min(right_pwm, self.max_pwm),
            -self.max_pwm
        )
        threshold = 25
        override = msg.buttons[7]
        if self.distance < threshold and override == 0:
            if left_pwm > 0 or right_pwm > 0:
                left_pwm = min(left_pwm, 0)
                right_pwm = min(right_pwm, 0)
        motor_msg.left_pwm = left_pwm
        motor_msg.right_pwm = right_pwm
        #SERVO ANALOG CONTROL
        bucket_axis = msg.axes[3]
        #deadzone
        if abs(bucket_axis) < 0.05:
            bucket_axis = 0.0
        #Incremental movement
        self.servo_angle += int(
            bucket_axis * 3
        )
        #Limit
        self.servo_angle = max(
            45,
            min(135, self.servo_angle)
        )
        motor_msg.servo_angle = (
            self.servo_angle
        )
        #PUBLISH
        self.publisher.publish(motor_msg)

    def distance_callback(self, msg):
        self.distance = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()