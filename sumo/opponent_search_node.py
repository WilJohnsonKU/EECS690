#!/usr/bin/env python3
# Description:
# This node is responsible for detecting, tracking, and pursuing the opponent robot in a sumo arena.
# It uses detection data from the orange_tracker node to locate the opponent, implements search
# patterns when no opponent is visible, and tracks and pursues the opponent when detected.
# It follows a behavior hierarchy, yielding to wall avoidance and neutral positioning when needed.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class OpponentSearchNode(Node):
    """
    Node for searching and tracking an opponent robot in a sumo arena.
    
    This node uses the detection data from the orange_tracker node to identify
    the opponent robot, implements search patterns when no opponent is visible,
    and tracks and pursues the opponent when detected. It follows a behavior
    hierarchy, yielding to wall avoidance and neutral positioning when needed.
    """
    
    def __init__(self):
        super().__init__('opponent_search_node')
        
        # Define parameters with default values
        self.declare_parameter('linear_speed_limit', 0.6)  # Maximum linear velocity (m/s)
        self.declare_parameter('angular_speed_limit', 2.0)  # Maximum angular velocity (rad/s)
        self.declare_parameter('search_pattern_interval', 5.0)  # Time for each search pattern (seconds)
        self.declare_parameter('camera_width', 640)  # Width of camera image in pixels
        self.declare_parameter('minimum_confidence', 0.5)  # Minimum detection confidence
        
        # Get parameter values
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').value
        self.search_interval = self.get_parameter('search_pattern_interval').value
        self.camera_width = self.get_parameter('camera_width').value
        self.minimum_confidence = self.get_parameter('minimum_confidence').value
        
        # State variables
        self.wall_avoidance_active = False   # Flag indicating if wall avoidance is active
        self.at_neutral_position = False     # Flag indicating if robot is at center
        self.opponent_detected = False       # Flag indicating if opponent is detected
        self.opponent_x = 0                  # X-coordinate of opponent in image
        self.opponent_y = 0                  # Y-coordinate of opponent in image 
        self.opponent_width = 0              # Width of opponent bounding box
        self.opponent_height = 0             # Height of opponent bounding box
        self.opponent_area = 0               # Area of opponent detection
        self.opponent_confidence = 0.0       # Detection confidence
        self.search_phase = 0                # Current phase of search pattern (0-3)
        self.last_pattern_change = self.get_clock().now()  # Time of last search pattern change
        
        # Based on wall_avoider.py and lidar_wall_avoider.py, we know how wall information is published
        # Subscribe to wall_info topic to determine if wall avoidance should be active
        self.wall_sub = self.create_subscription(
            Float32MultiArray,
            'wall_info',     # Topic from lidar_wall_avoider.py
            self.wall_info_callback,
            10)
        
        # Subscribe to neutral position status from neutral_position_node
        self.neutral_sub = self.create_subscription(
            Bool,
            'at_neutral_position',       # Topic from neutral_position_node
            self.neutral_position_callback,
            10)
        
        # The orange_tracker.py implements a RedBallFetcher that processes images but doesn't publish
        # detection data to a topic. Instead, it directly controls the robot.
        # 
        # For our needs, we should either:
        # 1. Modify orange_tracker.py to publish detection data (preferred)
        # 2. Subscribe to the camera image and do our own orange detection
        # 3. Use example/color_detect nodes as a reference
        #
        # For now, we'll implement direct image processing as a fallback since 
        # we can't modify orange_tracker.py in this task
        
        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',  # Same topic used by orange_tracker.py
            self.image_callback,
            10)
        
        # Create a bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',                   # Topic for velocity commands
            10)
        
        # Wall distance threshold (used to determine if wall avoidance should be active)
        self.declare_parameter('wall_distance_threshold', 0.5)  # meters
        self.wall_distance_threshold = self.get_parameter('wall_distance_threshold').value

        # Additional parameters for orange detection
        self.declare_parameter('min_orange_area', 500)  # Minimum area to consider a valid detection
        self.min_orange_area = self.get_parameter('min_orange_area').value
        
        # Timer for control loop at 10Hz
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Opponent Search Node has been started')
    
    def image_callback(self, msg):
        """
        Process camera images to detect orange objects.
        
        Args:
            msg (Image): Camera image
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert image to HSV color space for better color detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define orange color range in HSV
            # Based on orange_tracker.py but with corrected variable names
            lower_orange1 = np.array([0, 87, 186])
            upper_orange1 = np.array([10, 119, 255])
            lower_orange2 = np.array([49, 107, 173])
            upper_orange2 = np.array([74, 158, 255])
            
            # Create masks for the two orange ranges
            mask1 = cv2.inRange(hsv, lower_orange1, upper_orange1)
            mask2 = cv2.inRange(hsv, lower_orange2, upper_orange2)
            
            # Combine the masks
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Reset detection flag
            self.opponent_detected = False
            
            if contours:
                # Find the largest contour (assumed to be the opponent)
                cnt = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(cnt)
                
                if area > self.min_orange_area:
                    # Get the bounding circle
                    ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                    center = (int(x), int(y))
                    
                    # Get bounding rectangle for width and height
                    x, y, w, h = cv2.boundingRect(cnt)
                    
                    # Update detection data
                    self.opponent_detected = True
                    self.opponent_x = center[0]
                    self.opponent_y = center[1]
                    self.opponent_width = w
                    self.opponent_height = h
                    self.opponent_area = area
                    self.opponent_confidence = min(1.0, area / 10000.0)  # Simple confidence metric
                    
                    self.get_logger().debug(
                        f'Opponent detected at x={self.opponent_x}, y={self.opponent_y}, '
                        f'area={self.opponent_area}, conf={self.opponent_confidence:.2f}')
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
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
    
    def neutral_position_callback(self, msg):
        """
        Update neutral position status from neutral_position_node.
        
        Args:
            msg (Bool): Message indicating if robot is at neutral position
        """
        self.at_neutral_position = msg.data
    
    def control_loop(self):
        """
        Main control loop that runs at 10Hz to search for and pursue the opponent.
        
        This function implements a behavior hierarchy:
        1. Wall avoidance has highest priority
        2. Going to neutral position has second priority
        3. Opponent search/pursuit has lowest priority
        """
        # If wall avoidance is active, don't do anything - safety first!
        if self.wall_avoidance_active:
            self.get_logger().debug('Wall avoidance active - yielding control')
            return
        
        # If not at neutral position, let the neutral position node handle it
        if not self.at_neutral_position:
            self.get_logger().debug('Not at neutral position - yielding control')
            return
        
        # Create empty velocity command
        cmd = Twist()
        
        # Opponent tracking mode - use detection data from orange detection
        if self.opponent_detected:
            # Track and pursue the opponent
            img_center_x = self.camera_width / 2
            
            # Calculate normalized error (-1 to 1)
            normalized_error = (self.opponent_x - img_center_x) / img_center_x
            
            # Set angular velocity proportional to the error
            # Negative because robot should turn right if opponent is on the right
            angular_vel = -1.0 * normalized_error * self.angular_speed_limit * 0.7
            cmd.angular.z = min(self.angular_speed_limit, 
                              max(-self.angular_speed_limit, 
                                 angular_vel))
            
            # Set forward velocity based on detection size
            # Smaller area = further away = faster approach
            # Larger area = closer = slower approach
            
            # Calculate a speed factor based on area (inverse relationship)
            # The smaller the area, the faster we should move (up to max speed)
            area_factor = 1.0 - min(1.0, self.opponent_area / 50000.0)
            linear_vel = self.linear_speed_limit * area_factor
            
            # Ensure a minimum velocity to keep moving
            cmd.linear.x = max(0.1, linear_vel)
            
            self.get_logger().debug(
                f'Tracking opponent - area: {self.opponent_area}, '
                f'linear: {cmd.linear.x:.2f}, angular: {cmd.angular.z:.2f}')
        else:
            # Search pattern mode - implement search strategy when no opponent is visible
            current_time = self.get_clock().now()
            
            # Check if it's time to change the search pattern
            time_diff = (current_time - self.last_pattern_change).nanoseconds / 1e9
            if time_diff > self.search_interval:
                self.search_phase = (self.search_phase + 1) % 4
                self.last_pattern_change = current_time
                self.get_logger().info(f'Changing search pattern to phase {self.search_phase}')
            
            # Implement different search patterns
            if self.search_phase == 0:
                # Pattern 0: Spin in place clockwise
                cmd.angular.z = -self.angular_speed_limit * 0.5
                cmd.linear.x = 0.0
            elif self.search_phase == 1:
                # Pattern 1: Move forward
                cmd.linear.x = self.linear_speed_limit * 0.7
                cmd.angular.z = 0.0
            elif self.search_phase == 2:
                # Pattern 2: Spin in place counter-clockwise
                cmd.angular.z = self.angular_speed_limit * 0.5
                cmd.linear.x = 0.0
            elif self.search_phase == 3:
                # Pattern 3: Move in an arc (combined linear and angular motion)
                cmd.linear.x = self.linear_speed_limit * 0.5
                cmd.angular.z = self.angular_speed_limit * 0.3
            
            self.get_logger().debug(f'Searching - phase {self.search_phase}, linear: {cmd.linear.x:.2f}, angular: {cmd.angular.z:.2f}')
        
        # Publish velocity command to move the robot
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    """Main function to initialize the node and start processing."""
    rclpy.init(args=args)
    node = OpponentSearchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 