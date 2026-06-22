import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.get_logger().info("Odom Node Started")

        #CAR PHYSICAL PARAMETERS
        self.wheel_diameter = 0.043 #Smaller Diameter, Further IRL
        self.wheel_base = 0.1565 #Smaller Base, Smaller Angle IRL
        self.ticks_per_rev = 40.0
        self.max_tick_jump = 400
        self.distance_per_tick = (math.pi * self.wheel_diameter / self.ticks_per_rev)

        #INITIALIZATION
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        #SUBSCRIBE
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/encoder_ticks',
            self.encoder_callback,
            10
        )

        #PUBLISH
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        self.path_pub = self.create_publisher(
            Path,
            '/path',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

    #ENCODER CALLBACK
    def encoder_callback(self, msg):
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]
        #First message
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            return
        #DELTA TICKS
        delta_left_ticks = (
            left_ticks
            - self.prev_left_ticks
        )
        delta_right_ticks = (
            right_ticks
            - self.prev_right_ticks
        )

        #PROTECTION
        if (
            abs(delta_left_ticks) > self.max_tick_jump
            or
            abs(delta_right_ticks) > self.max_tick_jump
        ):
            self.get_logger().warn(
                f"ENCODER JUMP DETECTED "
                f"L:{delta_left_ticks} "
                f"R:{delta_right_ticks}"
            )
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            return

        #UPDATE PREVIOUS TICKS
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        #DELTA DISTANCE
        dl = (
            delta_left_ticks
            * self.distance_per_tick
        )
        dr = (
            delta_right_ticks
            * self.distance_per_tick
        )

        #DIFFERENTIAL DRIVE ODOM
        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.wheel_base
        theta_mid = self.theta + dtheta / 2.0
        self.x += dc * math.cos(theta_mid)
        self.y += dc * math.sin(theta_mid)
        self.theta += dtheta
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi

        #ODOM MESSAGE
        odom = Odometry()
        odom.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        #PATH MESSAGE
        pose = PoseStamped()
        pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = q
        self.path_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )
        self.path_msg.poses.append(pose)

        #PATH LIMITATION
        if len(self.path_msg.poses) > 1000:
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)

    #DEBUG
    def destroy_node(self):
        self.get_logger().info(
            f"Final Pose -> "
            f"x={self.x:.2f} "
            f"y={self.y:.2f} "
            f"theta={math.degrees(self.theta):.1f}"
        )
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()