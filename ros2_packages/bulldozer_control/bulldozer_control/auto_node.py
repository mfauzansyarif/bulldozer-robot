import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from bulldozer_interfaces.msg import MotorCommand
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
import json

class AutoNode(Node):
    def __init__(self):
        
        super().__init__('auto_node')

        #NAVIGATION PARAMETER
        self.turn_pwm = 60
        self.drive_pwm = 80 
        self.servo_angle = 90
        self.BUCKET_DUMP = 88
        self.BUCKET_CARRY = 135
        self.arrive_tolerance = 0.05
        self.heading_tolerance = math.radians(5)
        self.realign_tolerance = math.radians(45)
        self.obstacle_threshold = 15.0      
        
        #INITIALIZATION
        self.left_ticks = 0
        self.right_ticks = 0
        self.bucket_start_x = 0.0
        self.bucket_start_y = 0.0
        self.current_waypoint = 0
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0    
        self.avoid_step = 0
        self.avoid_start_time = 0.0
        self.avoid_start_x = 0.0
        self.avoid_start_y = 0.0
        self.distance = 999.0
        self.waypoints = []
        self.saved_mission = []
        self.auto_enabled = False
        self.markers_cleared = False
        self.loop_mission = False
        self.next_action = "MOVE"
        self.state = "ARRIVED"
        self.state_start_time = 0.0
        self.get_logger().info(
            "Auto Node Started"
        )

        #SUBSCRIBE (Distance, Auto Enable, Motor Command, Mission Loop, Goal Pose, Next Action, Encoder Ticks, Odom)
        self.distance_sub = self.create_subscription(
            Float32,
            '/distance',
            self.distance_callback,
            10
        )
        self.auto_sub = self.create_subscription(
            Bool,
            '/auto_enable',
            self.auto_callback,
            10
        )
        self.teleop_sub = self.create_subscription(
            MotorCommand,
            '/teleop_command',
            self.teleop_callback,
            10
        )
        self.loop_sub = self.create_subscription(
            Bool,
            "/mission_loop",
            self.loop_callback,
            10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.action_sub = self.create_subscription(
            String,
            "/next_action",
            self.action_callback,
            10
        )
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/encoder_ticks',
            self.encoder_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        #PUBLISHER (Auto Command, Robot Status, Waypoint Markers)
        self.motor_pub = self.create_publisher(
            MotorCommand,
            '/auto_command',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            "/robot_status",
            10
        )
        self.marker_pub = self.create_publisher(
            Marker,
            '/waypoints',
            10
        )

        #SERVICES (Clear Mission, Save Mission, Load Mission, Undo Waypoint)
        self.clear_srv = self.create_service(
            Empty,
            '/clear_mission',
            self.clear_mission_callback
        )
        self.save_srv = self.create_service(
            Empty,
            '/save_mission',
            self.save_mission_callback
        )
        self.load_srv = self.create_service(
            Empty,
            '/load_mission',
            self.load_mission_callback
        )
        self.undo_srv = self.create_service(
            Empty,
            "/undo_waypoint",
            self.undo_waypoint_callback
        )
        #TIMERS
        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )
        self.status_timer = self.create_timer(
            1.0,
            self.print_status
        )
        self.marker_timer = self.create_timer(
            0.1,
            self.publish_waypoints
        )
        self.gui_status_timer = self.create_timer(
            0.1,
            self.publish_status
        )

    # FSM STATES
    #
    # ARRIVED
    # TURN_TO_TARGET
    # GO_TO_TARGET
    # ALIGN_FINAL
    #
    # SCOOP_LOWER
    # SCOOP_FORWARD
    # SCOOP_RAISE
    # SCOOP_BACKWARD
    #
    # DUMP_LOWER
    # DUMP_RAISE
    #
    # AVOID_OBSTACLE
    def control_loop(self):

        if not self.auto_enabled:
            self.stop_robot()
            return
        if len(self.waypoints) == 0:
            self.stop_robot()
            return
        if self.current_waypoint >= len(self.waypoints):
            self.stop_robot()
            return
        
        motor_msg = MotorCommand()
        motor_msg.servo_angle = self.servo_angle
        self.target_x = (self.waypoints[self.current_waypoint][0])
        self.target_y = (self.waypoints[self.current_waypoint][1])
        self.target_theta = (self.waypoints[self.current_waypoint][2])
        current_action = (self.waypoints[self.current_waypoint][3])

        desired_heading = math.atan2(
            self.target_y - self.y,
            self.target_x - self.x
        )
        heading_error = self.normalize_angle(
            desired_heading - self.theta
        )
        distance_error = math.sqrt(
            (self.target_x - self.x) ** 2 +
            (self.target_y - self.y) ** 2
        )

        now = time.time()

        #TURN TO TARGET
        if self.state == "TURN_TO_TARGET":
            if abs(heading_error) < self.heading_tolerance:
                self.state = "GO_TO_TARGET"
                self.get_logger().info("GO_TO_TARGET")
            else:
                turn_pwm = self.turn_pwm
                if heading_error > 0:
                    motor_msg.left_pwm = -turn_pwm
                    motor_msg.right_pwm = turn_pwm
                else:
                    motor_msg.left_pwm = turn_pwm
                    motor_msg.right_pwm = -turn_pwm

        #GO TO TARGET
        elif self.state == "GO_TO_TARGET":
            if (
                self.servo_angle != self.BUCKET_CARRY
                and
                self.distance < self.obstacle_threshold
            ):
                self.state = "AVOID_OBSTACLE"
                self.avoid_step = 0
                self.avoid_start_time = time.time()
                self.get_logger().info("OBSTACLE DETECTED")
                motor_msg.servo_angle = self.servo_angle
                self.motor_pub.publish(motor_msg)
                return
            if distance_error < self.arrive_tolerance:
                self.state = "ALIGN_FINAL"
                self.get_logger().info("ALIGN_FINAL")
            elif abs(heading_error) > self.realign_tolerance:
                self.state = "TURN_TO_TARGET"
            else:
                motor_msg.left_pwm = self.drive_pwm
                motor_msg.right_pwm = self.drive_pwm

        #AVOID OBSTACLE
        elif self.state == "AVOID_OBSTACLE":
            #1 BACKWARD
            if self.avoid_step == 0:
                motor_msg.left_pwm = -self.drive_pwm
                motor_msg.right_pwm = -self.drive_pwm
                if (time.time() - self.avoid_start_time) > 1.0:
                    self.avoid_step = 1
                    self.avoid_start_time = time.time()
            #2 RIGHT TURN
            elif self.avoid_step == 1:
                motor_msg.left_pwm = self.turn_pwm
                motor_msg.right_pwm = -self.turn_pwm
                if (time.time() - self.avoid_start_time) > 1.0:
                    self.avoid_step = 2
                    self.avoid_start_x = self.x
                    self.avoid_start_y = self.y
                    self.avoid_start_time = time.time()
            #3 FORWARD
            elif self.avoid_step == 2:
                motor_msg.left_pwm = self.drive_pwm
                motor_msg.right_pwm = self.drive_pwm
                avoid_distance = math.sqrt(
                    (self.x - self.avoid_start_x)**2 +
                    (self.y - self.avoid_start_y)**2
                )
                if avoid_distance > 0.20:
                    self.state = "TURN_TO_TARGET"
                    self.avoid_step = 0
                    self.get_logger().info("REJOIN WAYPOINT")

        #ALIGN FINAL
        elif self.state == "ALIGN_FINAL":
            final_error = self.normalize_angle(self.target_theta - self.theta)
            if abs(final_error) < math.radians(5):
                if current_action == "SCOOP":
                    self.state = "SCOOP_LOWER"
                    self.state_start_time = time.time()
                elif current_action == "DUMP":
                    self.state = "DUMP_LOWER"
                    self.state_start_time = time.time()
                else:
                    self.current_waypoint += 1
                    if self.current_waypoint >= len(self.waypoints):
                        if self.loop_mission:
                            self.waypoints = list(self.saved_mission)
                            self.current_waypoint = 0
                            self.state = "TURN_TO_TARGET"
                            self.get_logger().info("MISSION LOOP RESTART")
                        else:
                            self.state = "ARRIVED"
                            self.waypoints.clear()
                            self.current_waypoint = 0
                            self.publish_waypoints()
                    else:
                        self.state = "TURN_TO_TARGET"
            else:
                if final_error > 0:
                    motor_msg.left_pwm = -self.turn_pwm
                    motor_msg.right_pwm = self.turn_pwm
                else:
                    motor_msg.left_pwm = self.turn_pwm
                    motor_msg.right_pwm = -self.turn_pwm

        #SCOOP LOWER
        elif self.state == "SCOOP_LOWER":
            self.servo_angle = self.BUCKET_DUMP
            if (time.time() - self.state_start_time) > 1.0:
                self.bucket_start_x = self.x
                self.bucket_start_y = self.y
                self.state = "SCOOP_FORWARD"
                self.get_logger().info("SCOOP_FORWARD")

        #SCOOP FORWARD
        elif self.state == "SCOOP_FORWARD":
            self.servo_angle = self.BUCKET_DUMP
            motor_msg.left_pwm = self.drive_pwm
            motor_msg.right_pwm = self.drive_pwm
            scoop_distance = math.sqrt(
                (self.x - self.bucket_start_x)**2 +
                (self.y - self.bucket_start_y)**2
            )
            if scoop_distance > 0.15:
                self.state = "SCOOP_RAISE"
                self.state_start_time = time.time()

        #SCOOP RAISE
        elif self.state == "SCOOP_RAISE":
            self.servo_angle = self.BUCKET_CARRY
            if (time.time() - self.state_start_time) > 1.0:
                self.bucket_start_x = self.x
                self.bucket_start_y = self.y
                self.state = "SCOOP_BACKWARD"

        #SCOOP BACKWARD
        elif self.state == "SCOOP_BACKWARD":
            self.servo_angle = self.BUCKET_CARRY
            motor_msg.left_pwm = -self.drive_pwm
            motor_msg.right_pwm = -self.drive_pwm
            back_distance = math.sqrt(
                (self.x - self.bucket_start_x)**2 +
                (self.y - self.bucket_start_y)**2
            )
            if back_distance > 0.15:
                self.current_waypoint += 1
                if self.current_waypoint >= len(self.waypoints):
                    if self.loop_mission:
                        self.waypoints = list(self.saved_mission)
                        self.current_waypoint = 0
                        self.state = "TURN_TO_TARGET"
                        self.get_logger().info("MISSION LOOP RESTART")
                    else:
                        self.state = "ARRIVED"
                        self.waypoints.clear()
                        self.current_waypoint = 0
                        self.publish_waypoints()
                    self.publish_waypoints()
                else:
                    self.state = "TURN_TO_TARGET"
                self.get_logger().info("SCOOP COMPLETE")

        #DUMP LOWER
        elif self.state == "DUMP_LOWER":
            self.servo_angle = self.BUCKET_DUMP
            if (time.time() - self.state_start_time) > 1.0:
                self.state = "DUMP_RAISE"
                self.state_start_time = time.time()

        #DUMP RAISE
        elif self.state == "DUMP_RAISE":
            self.servo_angle = self.BUCKET_CARRY
            if (time.time() - self.state_start_time) > 1.0:
                self.current_waypoint += 1
                if self.current_waypoint >= len(self.waypoints):
                    if self.loop_mission:
                        self.waypoints = list(self.saved_mission)
                        self.current_waypoint = 0
                        self.state = "TURN_TO_TARGET"
                        self.get_logger().info("MISSION LOOP RESTART")
                    else:
                        self.state = "ARRIVED"
                        self.waypoints.clear()
                        self.current_waypoint = 0
                        self.publish_waypoints()
                    self.publish_waypoints()
                else:
                    self.state = "TURN_TO_TARGET"
                self.get_logger().info("DUMP COMPLETE")

        #ARRIVED
        elif self.state == "ARRIVED":
            motor_msg.left_pwm = 0
            motor_msg.right_pwm = 0
            self.servo_angle = self.BUCKET_CARRY

        #PUBLISH
        motor_msg.servo_angle = self.servo_angle
        self.motor_pub.publish(motor_msg)
    
    #STOP ROBOT
    def stop_robot(self):
        motor_msg = MotorCommand()
        motor_msg.left_pwm = 0
        motor_msg.right_pwm = 0
        motor_msg.servo_angle = self.servo_angle
        self.motor_pub.publish(motor_msg)

    #NORMALIZE ANGLE
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    #STATUS
    def print_status(self):
            desired_heading = math.atan2(
                self.target_y - self.y,
                self.target_x - self.x
            )
            heading_error = self.normalize_angle(
                desired_heading - self.theta
            )
            distance_error = math.sqrt(
                (self.target_x - self.x)**2 +
                (self.target_y - self.y)**2
            )
            self.get_logger().info(
                f"WP:{self.current_waypoint}/"
                f"{len(self.waypoints)} "
                f"State:{self.state} "
                f"X:{self.x:.2f} "
                f"Y:{self.y:.2f} "
                f"Theta:{math.degrees(self.theta):.1f} "
                f"HeadErr:{math.degrees(heading_error):.1f} "
                f"DistErr:{distance_error:.2f}"
            )
    
    #PUBLISH WAYPOINTS
    def publish_waypoints(self):
        #NO WAYPOINT
        if len(self.waypoints) == 0:
            if not self.markers_cleared:
                delete_all = Marker()
                delete_all.header.frame_id = "odom"
                delete_all.header.stamp = (self.get_clock().now().to_msg())
                delete_all.action = Marker.DELETEALL
                self.marker_pub.publish(delete_all)
                self.markers_cleared = True
            return
        #WAYPOINTS
        self.markers_cleared = False
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = (self.get_clock().now().to_msg())
            marker.ns = "waypoints"
            marker.id = i + 100
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = (float(wp[0]))
            marker.pose.position.y = (float(wp[1]))
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            #ACTIVE WAYPOINT
            if i == self.current_waypoint:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            #NON-ACTIVE WAYPOINT
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "odom"
            arrow_marker.header.stamp = (self.get_clock().now().to_msg())
            arrow_marker.ns = "waypoint_arrows"
            arrow_marker.id = i + 2000
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = float(wp[0])
            arrow_marker.pose.position.y = float(wp[1])
            arrow_marker.pose.position.z = 0.02
            theta = float(wp[2])
            arrow_marker.pose.orientation.z = math.sin(theta / 2.0)
            arrow_marker.pose.orientation.w = math.cos(theta / 2.0)
            arrow_marker.scale.x = 0.10
            arrow_marker.scale.y = 0.02
            arrow_marker.scale.z = 0.02

            if i == self.current_waypoint:
                arrow_marker.color.r = 0.0
                arrow_marker.color.g = 1.0
                arrow_marker.color.b = 0.0
            else:
                arrow_marker.color.r = 1.0
                arrow_marker.color.g = 1.0
                arrow_marker.color.b = 0.0
            arrow_marker.color.a = 1.0
            self.marker_pub.publish(arrow_marker)
            #LABEL
            text_marker = Marker()
            text_marker.header.frame_id = "odom"
            text_marker.header.stamp = (self.get_clock().now().to_msg())
            text_marker.ns = "waypoint_labels"
            text_marker.id = i + 1000
            text_marker.type = (Marker.TEXT_VIEW_FACING)
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = (float(wp[0]))
            text_marker.pose.position.y = (float(wp[1]))
            text_marker.pose.position.z = 0.08
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.05
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            action = wp[3]
            text_marker.text = (f"WP{i}\n{action}")
            self.marker_pub.publish(text_marker)
    
    #PUBLISH STATUS
    def publish_status(self):
        msg = String()
        msg.data = (
            f"AUTO : {self.auto_enabled}\n"
            f"LOOP : {self.loop_mission}\n"
            f"STATE : {self.state}\n"
            f"WP : {self.current_waypoint}/{len(self.waypoints)}\n"
            f"DIST : {self.distance:.1f} cm\n"
            f"SERVO : {self.servo_angle}\n"
            f"ACTION : {self.next_action}"
        )
        self.status_pub.publish(msg)

    #CALLBACK GOAL
    def goal_callback(self, msg):
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        q = msg.pose.orientation
        goal_theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )
        self.waypoints.append(
            (
                goal_x,
                goal_y,
                goal_theta,
                self.next_action
            )
        )
        self.get_logger().info(
            f"ADD GOAL "
            f"{goal_x:.2f}, "
            f"{goal_y:.2f}, "
            f"{math.degrees(goal_theta):.1f}deg, "
            f"{self.next_action}"
        )
        if self.state == "ARRIVED":
            self.current_waypoint = 0
            self.state = "TURN_TO_TARGET"
            self.get_logger().info(
                "START MISSION"
            )
        self.saved_mission = list(
            self.waypoints
        )

    #CALLBACK DISTANCE
    def distance_callback(self, msg):
        self.distance = msg.data
    
    #CALLBACK TELEOP
    def teleop_callback(self, msg):
        if not self.auto_enabled:
            self.servo_angle = msg.servo_angle
    
    #CALLBACK AUTO
    def auto_callback(self, msg):
        self.auto_enabled = msg.data
        self.get_logger().info(f"AUTO: {self.auto_enabled}")

    #CALLBACK ENCODER
    def encoder_callback(self, msg):
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]

    #CALLBACK ODOM
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    #CALLBACK CLEAR MISSION
    def clear_mission_callback(self,request,response):
        self.waypoints.clear()
        self.current_waypoint = 0
        self.state = "ARRIVED"
        self.markers_cleared = False
        self.publish_waypoints()
        self.get_logger().info("MISSION CLEARED")
        return response

    #CALLBACK SAVE MISSION
    def save_mission_callback(self,request,response):
        with open("mission.json","w") as f:
            json.dump(self.waypoints,f,indent=4)
            self.saved_mission = list(self.waypoints)
        self.get_logger().info("MISSION SAVED")
        return response
    
    #CALLBACK LOAD MISSION
    def load_mission_callback(self,request,response):
        with open("mission.json","r") as f:
            self.waypoints = json.load(f)
            self.saved_mission = list(self.waypoints)
        self.current_waypoint = 0
        self.markers_cleared = False
        if self.auto_enabled:
            self.state = "TURN_TO_TARGET"
        else:
            self.state = "ARRIVED"
        self.get_logger().info("MISSION LOADED")
        self.publish_waypoints()
        return response
    
    #CALLBACK ACTION
    def action_callback(self, msg):
        self.next_action = msg.data
        self.get_logger().info(f"NEXT ACTION: {self.next_action}")

    #CALLBACK LOOP
    def loop_callback(self, msg):
        self.loop_mission = msg.data
        self.get_logger().info(f"MISSION LOOP: "f"{self.loop_mission}")

    #CALLBACK UNDO
    def undo_waypoint_callback(self,request,response):
            if len(self.waypoints) > 0:
                self.waypoints.pop()
                if self.current_waypoint >= len(self.waypoints):
                    self.current_waypoint = max(0,len(self.waypoints) - 1)
                self.saved_mission = list(self.waypoints)
                self.markers_cleared = False
                self.publish_waypoints()
                self.get_logger().info("UNDO WAYPOINT")
            if len(self.waypoints) == 0:
                self.state = "ARRIVED"
            else:
                self.state = "TURN_TO_TARGET"
            return response

def main(args=None):
    rclpy.init(args=args)
    node = AutoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()