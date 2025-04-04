#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        # Subscribe to the centroid information
        self.centroid_sub = self.create_subscription(
            Float32MultiArray,
            'centroid_info',
            self.centroid_callback,
            qos_profile
        )
        # Subscribe to the active node topic to check if we should be active
        self.active_node_sub = self.create_subscription(
            String,
            '/sumo/active_node',
            self.active_node_callback,
            1
        )
        # Publisher to drive the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1) # /controller/cmd_vel is faster??

        # Publisher for at Center flag
        self.at_center_publisher = self.create_publisher(
            Bool,
            '/sumo/at_center',
            1
        )

        # Store the latest centroid info and whether safety node is active.
        # Centroid info is expected as [room_center_x, room_center_y] in the robot's coordinate frame.
        self.latest_centroid = None
        self.active = False

        # Tolerance for stopping adjustments.
        self.distance_tolerance = 0.125  # meters

        # Run control loop at 10 Hz.
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Safety Node started. Waiting for centroid info and activation.")

        self.at_center = False
        self.IS_SAFETY = False

    def active_node_callback(self, msg):
        # Activate safety behavior if the active node is set to a safety state.
        if msg.data in ["neutral_position", "safety_node"]:
            self.IS_SAFETY = msg.data == "safety_node"
            self.active = True
        else:
            self.active = False

    def centroid_callback(self, msg):
        if len(msg.data) == 2:
            self.latest_centroid = msg.data
            # self.get_logger().info(f"Centroid info updated: {msg.data}")
        else:
            self.get_logger().error("Malformed centroid info received.")

    def timer_callback(self):
        twist = Twist()
        if self.latest_centroid is not None:
            # Use the room center (centroid) as our target.
            room_center_x, room_center_y = self.latest_centroid
            # Error vector from robot to target.
            error_x = room_center_x
            error_y = room_center_y
            distance_error = math.hypot(error_x, error_y)

            # If we're within tolerance, stop moving.
            if distance_error < self.distance_tolerance:
                twist.linear.x = 0.0
                twist.linear.y = 0.0

                msg = Bool()
                msg.data = True
                self.at_center_publisher.publish(msg)
                
                self.at_center = True

                if self.active:
                    # Override angular.z with the stored value from the cmd_vel subscriber.
                    self.cmd_vel_pub.publish(twist)
                    self.active = False
            else:
                if self.at_center:
                    self.at_center = False
                    msg = Bool()
                    msg.data = False
                    self.at_center_publisher.publish(msg)

                # Use a proportional controller to command velocities in x and y.
                if self.active:
                    K_linear = 1.0
                    twist.linear.x = K_linear * error_x
                    twist.linear.y = K_linear * error_y
                    self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
