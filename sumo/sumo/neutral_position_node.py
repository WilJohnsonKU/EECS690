#!/usr/bin/env python3
# Description:
# This node is responsible for positioning the robot at the center of the 5x5 foot sumo arena.
# It subscribes to odometry data to determine the robot's current position and publishes
# velocity commands to guide the robot to the center of the arena. It yields control to
# wall avoidance behaviors when they are active, prioritizing safety.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
import math
import numpy as np

class NeutralPositionNode(Node):
    """
    Node responsible for positioning the robot at the center of a sumo arena.
    
    This node monitors the robot's position and guides it to the center of the arena
    when needed, while yielding control to higher priority behaviors like wall avoidance.
    """
    
    def __init__(self):
        super().__init__('neutral_position_node')
        
        # Define parameters with default values
        self.declare_parameter('arena_center_x', 0.0)     # X-coordinate of arena center (meters)
        self.declare_parameter('arena_center_y', 0.0)     # Y-coordinate of arena center (meters)
        self.declare_parameter('position_tolerance', 0.1) # How close to center is considered "at center" (meters)
        self.declare_parameter('linear_speed_limit', 0.6) # Maximum linear velocity (m/s)
        self.declare_parameter('angular_speed_limit', 2.0) # Maximum angular velocity (rad/s)
        
        # Get parameter values
        self.arena_center_x = self.get_parameter('arena_center_x').value
        self.arena_center_y = self.get_parameter('arena_center_y').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').value
        
        # State variables
        self.current_x = 0.0        # Current X position of robot (meters)
        self.current_y = 0.0        # Current Y position of robot (meters)
        self.current_yaw = 0.0      # Current heading of robot (radians)
        self.wall_avoidance_active = False  # Flag indicating if wall avoidance is active
        self.at_neutral_position = False    # Flag indicating if robot is at center
        
        # Subscriber for odometry data
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',                # Topic for robot odometry 
            self.odom_callback,     # Callback function
            10)                     # Queue size
        
        # Subscriber for wall avoidance status
        # Based on pattern from square_controller.py, we will subscribe to wall info
        # and determine wall avoidance status based on distance threshold
        self.wall_sub = self.create_subscription(
            Float32MultiArray,
            'wall_info',            # Topic from lidar_wall_avoider.py
            self.wall_info_callback,
            10)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',              # Topic for velocity commands
            10)
        
        # Publisher for neutral position status
        self.neutral_status_pub = self.create_publisher(
            Bool,
            'at_neutral_position',  # Topic to indicate if robot is at center
            10)
        
        # Wall distance threshold (used to determine if wall avoidance should be active)
        self.declare_parameter('wall_distance_threshold', 0.5)  # meters
        self.wall_distance_threshold = self.get_parameter('wall_distance_threshold').value
        
        # Timer for control loop at 10Hz
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Neutral Position Node has been started')
    
    def odom_callback(self, msg):
        """
        Process odometry data to update the robot's current position and orientation.
        
        Args:
            msg (Odometry): The odometry message containing position and orientation
        """
        # Extract position data from odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw (heading) from quaternion
        q = msg.pose.pose.orientation
        # Convert quaternion to Euler angle (yaw)
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
    
    def wall_info_callback(self, msg):
        """
        Process wall information to determine if wall avoidance should be active.
        
        Args:
            msg (Float32MultiArray): Message containing [distance_to_wall, angle_to_wall]
        """
        if len(msg.data) >= 1:
            distance_to_wall = msg.data[0]
            # Wall avoidance is active if we're too close to a wall
            self.wall_avoidance_active = distance_to_wall < self.wall_distance_threshold
            if self.wall_avoidance_active:
                self.get_logger().debug(f'Wall avoidance active: distance={distance_to_wall:.2f}m')
    
    def control_loop(self):
        """
        Main control loop that runs at 10Hz to navigate the robot to the arena center.
        This function calculates the required velocities and publishes them if appropriate.
        """
        # Calculate distance to arena center
        dx = self.arena_center_x - self.current_x
        dy = self.arena_center_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Determine if robot is at the neutral position (center of arena)
        is_at_neutral = distance < self.position_tolerance
        self.at_neutral_position = is_at_neutral
        
        # Publish neutral position status for other nodes to use
        status_msg = Bool()
        status_msg.data = is_at_neutral
        self.neutral_status_pub.publish(status_msg)
        
        # Don't move if wall avoidance is active - safety first!
        if self.wall_avoidance_active:
            self.get_logger().debug('Wall avoidance active - yielding control')
            return
        
        # Create empty velocity command
        cmd = Twist()
        
        # Only need to move if not at neutral position
        if not is_at_neutral:
            # Calculate angle to target (arena center)
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference (shortest path)
            angle_diff = target_angle - self.current_yaw
            # Normalize angle to range [-pi, pi]
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Set angular velocity proportional to angle difference
            # Limit to angular_speed_limit
            cmd.angular.z = min(self.angular_speed_limit, 
                              max(-self.angular_speed_limit, 
                                 angle_diff))
            
            # Only move forward if roughly facing the target
            if abs(angle_diff) < 0.5:  # ~30 degrees
                # Set linear velocity proportional to distance
                # Limit to linear_speed_limit
                cmd.linear.x = min(self.linear_speed_limit, 
                                 max(-self.linear_speed_limit,
                                     distance))
                self.get_logger().debug(f'Moving to center: distance={distance:.2f}m, angle_diff={angle_diff:.2f}rad')
            else:
                # Just turn to face the center if not aligned
                self.get_logger().debug(f'Turning to face center: angle_diff={angle_diff:.2f}rad')
        else:
            # At neutral position, stop moving
            self.get_logger().debug('At neutral position')
        
        # Publish velocity command to move the robot
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    """Main function to initialize the node and start processing."""
    rclpy.init(args=args)
    node = NeutralPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 