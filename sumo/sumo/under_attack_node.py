import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float32
import numpy as np
from collections import deque

class AttackDetectorNode(Node):
    """
    Node that detects when the robot is being pushed or attacked.
    
    Uses IMU accelerometer data combined with commanded velocity to distinguish
    between expected movement and external forces like pushing or collisions.
    """
    
    def __init__(self):
        # Initialize the ROS node
        super().__init__('attack_detector_node')
        
        # Subscribe to IMU data
        self.imu_subscription = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
            
        # Subscribe to commanded velocity to distinguish intended vs. external motion
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            
        # Publisher for attack detection status
        self.attack_publisher = self.create_publisher(
            Bool, '/attack_detected', 10)
            
        # Publisher for current acceleration (for debugging)
        self.accel_publisher = self.create_publisher(
            Float32, '/current_acceleration', 10)
            
        # Parameters with default values
        self.declare_parameter('accel_threshold', 1.5)         # m/s² threshold to detect attack
        self.declare_parameter('detection_window', 5)          # Number of samples in detection window
        self.declare_parameter('confidence_threshold', 0.6)    # Required confidence to trigger detection
        self.declare_parameter('filter_alpha', 0.3)            # Low-pass filter coefficient
        self.declare_parameter('angular_threshold', 0.7)       # rad/s threshold for rotation detection
        self.declare_parameter('attack_cooldown', 1.0)         # Seconds before new attack can be detected
        
        # Get parameter values
        self.accel_threshold = self.get_parameter('accel_threshold').value
        self.detection_window = self.get_parameter('detection_window').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.filter_alpha = self.get_parameter('filter_alpha').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.attack_cooldown = self.get_parameter('attack_cooldown').value
        
        # State tracking
        self.attack_detected = False
        self.commanded_linear_x = 0.0
        self.commanded_angular_z = 0.0
        
        # Buffer for recent acceleration readings
        self.accel_buffer = deque(maxlen=self.detection_window)
        
        # Filtered acceleration values
        self.filtered_accel_x = 0.0
        self.filtered_accel_y = 0.0
        self.filtered_angular_z = 0.0
        
        # Create a timer for debouncing and handling attack state
        self.attack_timer = None
        
        # Log startup information
        self.get_logger().info(f"Attack Detector started with threshold={self.accel_threshold} m/s²")
        self.get_logger().info(f"Detection window: {self.detection_window} samples, " 
                               f"Confidence threshold: {self.confidence_threshold*100}%")
    
    def cmd_vel_callback(self, msg):
        """
        Store commanded velocities to distinguish between intended and external motion.
        
        Args:
            msg (Twist): The commanded velocity message
        """
        self.commanded_linear_x = msg.linear.x
        self.commanded_angular_z = msg.angular.z
    
    def imu_callback(self, msg):
        """
        Process IMU data to detect attacks/pushes.
        
        Args:
            msg (Imu): The IMU message containing acceleration and angular velocity
        """
        # Extract acceleration values
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        angular_z = msg.angular_velocity.z
        
        # Apply low-pass filter to reduce noise
        self.filtered_accel_x = self.filter_alpha * accel_x + (1 - self.filter_alpha) * self.filtered_accel_x
        self.filtered_accel_y = self.filter_alpha * accel_y + (1 - self.filter_alpha) * self.filtered_accel_y
        self.filtered_angular_z = self.filter_alpha * angular_z + (1 - self.filter_alpha) * self.filtered_angular_z
        
        # Calculate horizontal acceleration magnitude
        accel_mag = np.sqrt(self.filtered_accel_x**2 + self.filtered_accel_y**2)
        
        # Publish current acceleration (for debugging/visualization)
        accel_msg = Float32()
        accel_msg.data = float(accel_mag)
        self.accel_publisher.publish(accel_msg)
        
        # Store in buffer
        self.accel_buffer.append({
            'magnitude': accel_mag,
            'x': self.filtered_accel_x,
            'y': self.filtered_accel_y,
            'angular_z': self.filtered_angular_z,
            'expected': self.is_acceleration_expected(accel_mag, self.filtered_angular_z)
        })
        
        # Run detection logic
        self.detect_attack()
    
    def is_acceleration_expected(self, accel_mag, angular_z):
        """
        Determine if current acceleration is expected based on commanded motion.
        
        Args:
            accel_mag (float): The current acceleration magnitude
            angular_z (float): The current angular velocity around z-axis
            
        Returns:
            bool: True if the acceleration is expected, False otherwise
        """
        # If we're commanding significant forward/backward motion
        if abs(self.commanded_linear_x) > 0.1:
            # Check if acceleration is in the same direction as commanded
            expected_sign = np.sign(self.commanded_linear_x) 
            actual_sign = np.sign(self.filtered_accel_x)
            
            # If signs match (both positive or both negative), this is expected
            if expected_sign == actual_sign:
                return True
                
        # If we're commanding significant rotation
        if abs(self.commanded_angular_z) > 0.2:
            # A certain amount of centripetal acceleration is expected during rotation
            return True
            
        # Low acceleration during no movement is normal (minor vibrations)
        if accel_mag < 0.5 and abs(angular_z) < 0.3:
            return True
            
        return False
    
    def detect_attack(self):
        """
        Sophisticated attack detection algorithm using confidence-based assessment.
        """
        if len(self.accel_buffer) < self.detection_window:
            return  # Not enough data yet
            
        # Count how many samples exceed threshold and are unexpected
        attack_count = 0
        high_accel_count = 0
        high_angular_count = 0
        
        for sample in self.accel_buffer:
            # Check for high unexpected linear acceleration
            if sample['magnitude'] > self.accel_threshold and not sample['expected']:
                high_accel_count += 1
                attack_count += 1
                
            # Check for high unexpected angular velocity
            if (abs(sample['angular_z']) > self.angular_threshold and 
                abs(self.commanded_angular_z) < 0.1):
                high_angular_count += 1
                attack_count += 1
        
        # Calculate confidence level - divide by 2*window_size because we're checking two conditions
        confidence = attack_count / (2 * len(self.accel_buffer))
        
        # Determine if under attack based on confidence
        is_under_attack = confidence >= self.confidence_threshold
        
        # Handle state change
        if is_under_attack and not self.attack_detected:
            self.attack_detected = True
            
            # Detailed logging of detection
            self.get_logger().info(f"Attack detected! Confidence: {confidence:.2f}")
            self.get_logger().info(f"High accel: {high_accel_count}/{self.detection_window}, " 
                                  f"High angular: {high_angular_count}/{self.detection_window}")
            self.get_logger().info(f"Avg Acceleration: {np.mean([s['magnitude'] for s in self.accel_buffer]):.2f} m/s²")
            
            # Publish detection result
            msg = Bool()
            msg.data = True
            self.attack_publisher.publish(msg)
            
            # Start cooldown timer
            if self.attack_timer is not None:
                self.attack_timer.cancel()
            self.attack_timer = self.create_timer(self.attack_cooldown, self.reset_attack_state)
            
        elif not is_under_attack and self.attack_detected and self.attack_timer is None:
            # Only reset directly if timer isn't active
            self.reset_attack_state()
    
    def reset_attack_state(self):
        """Reset attack state after cooldown period."""
        if self.attack_timer is not None:
            self.attack_timer.cancel()
            self.attack_timer = None
            
        if self.attack_detected:
            self.attack_detected = False
            self.get_logger().info("Attack ended")
            
            # Publish end of attack
            msg = Bool()
            msg.data = False
            self.attack_publisher.publish(msg)

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = AttackDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
