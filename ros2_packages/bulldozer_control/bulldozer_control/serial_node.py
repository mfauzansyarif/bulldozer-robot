import rclpy
import socket
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from bulldozer_interfaces.msg import MotorCommand

class UDPNode(Node):
    def __init__(self):

        super().__init__('udp_node')

        self.get_logger().info(
            "UDP Node Started"
        )

        #ESP32 IP PARAMETER
        self.esp32_ip = "172.20.10.2"
        self.udp_port = 8888

        #UDP SOCKET
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )
        self.sock.bind(
            ('0.0.0.0', 8888)
        )

        self.sock.setblocking(False)

        #LATEST COMMAND
        self.latest_command = "0,0,90"

        #DISTANCE PUBLISHER
        self.distance_pub = self.create_publisher(
            Float32,
            '/distance',
            10
        )
        #RPM PUBLISHER
        self.rpm_pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_rpm',
            10
        )
        #ENCODER PUBLISHER
        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/encoder_ticks',
            10
        )
        #MOTOR SUBSCRIBER
        self.subscription = self.create_subscription(
            MotorCommand,
            '/motor_command',
            self.motor_callback,
            10
        )

        #SEND TIMER
        self.send_timer = self.create_timer(
            0.05,
            self.send_command
        )
        #RECEIVE TIMER
        self.receive_timer = self.create_timer(
            0.01,
            self.receive_udp
        )

    #MOTOR CALLBACK
    def motor_callback(self, msg):
        self.latest_command = (
            f"{msg.left_pwm},"
            f"{msg.right_pwm},"
            f"{msg.servo_angle}"
        )

    #SEND COMMAND
    def send_command(self):
        try:
            self.sock.sendto(
                self.latest_command.encode(),
                (
                    self.esp32_ip,
                    self.udp_port
                )
            )
        except Exception as e:
            self.get_logger().warn(
                f"UDP Send Error: {e}"
            )

    #RECEIVE UDP
    def receive_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            line = (
                data.decode(errors='ignore')
                .strip()
            )
            #DISTANCE
            if line.startswith("DIST:"):
                distance = float(
                    line.replace("DIST:", "")
                )
                msg = Float32()
                msg.data = distance
                self.distance_pub.publish(msg)
            #RPM
            elif line.startswith("RPM:"):
                rpm_data = (
                    line.replace("RPM:", "")
                    .split(",")
                )
                if len(rpm_data) != 2:
                    return
                left_rpm = float(rpm_data[0])
                right_rpm = float(rpm_data[1])
                msg = Float32MultiArray()
                msg.data = [
                    left_rpm,
                    right_rpm
                ]
                self.rpm_pub.publish(msg)
            #ENCODER
            elif line.startswith("ENC:"):
                enc_data = (
                    line.replace("ENC:", "")
                    .split(",")
                )
                if len(enc_data) != 2:
                    return
                left_ticks = int(enc_data[0])
                right_ticks = int(enc_data[1])
                msg = Int32MultiArray()
                msg.data = [
                    left_ticks,
                    right_ticks
                ]
                self.encoder_pub.publish(msg)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = UDPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()