import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class EscapeNode(Node):
    def __init__(self):
        # Initialize the node with the name 'escape_node'
        super().__init__('escape_node')
        
        # Publisher for velocity commands (to control robot movement)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscription to LiDAR data from the '/scan_raw' topic
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        
        # Subscription to opponent proximity data (expects [distance, angle] in Float32MultiArray)
        self.opponent_sub = self.create_subscription(Float32MultiArray, 'opponent_info', self.opponent_callback, 10)
        
        # Internal state flags
        self.near_wall = False      # True if a wall is detected within a threshold distance
        self.opponent_nearby = False  # True if an opponent is detected within a threshold distance
        
        # Distance thresholds (in meters) for triggering escape maneuvers
        self.wall_threshold = 0.4      # Distance below which a wall is considered too close
        self.opponent_threshold = 0.7  # Distance below which an opponent is considered too close
        
        # Create a timer that calls the escape processing function every 0.1 seconds
        self.create_timer(0.1, self.process_escape)
        
        self.get_logger().info("EscapeNode initialized. Ready to monitor obstacles and opponents.")

    def lidar_callback(self, msg):
        """
        Processes LiDAR data to detect nearby walls.
        Uses selected slices of the LiDAR ranges (front, left, right) and sets the 'near_wall' flag
        if any distance is below the wall threshold.
        """
        # Front: combine the first and last 10 readings (assuming symmetrical LiDAR distribution)
        front = min(msg.ranges[0:10] + msg.ranges[-10:])
        # Left: use a slice from the left-hand side (adjust indices as needed)
        left = min(msg.ranges[60:100])
        # Right: use a slice from the right-hand side (adjust indices as needed)
        right = min(msg.ranges[-100:-60])
        
        # Set flag if any of the readings indicate a wall is too close
        self.near_wall = (front < self.wall_threshold or left < self.wall_threshold or right < self.wall_threshold)

    def opponent_callback(self, msg):
        """
        Processes opponent information.
        Expects the first element of 'msg.data' to be the distance.
        Sets the 'opponent_nearby' flag if the opponent is closer than the set threshold.
        """
        distance = msg.data[0]
        self.opponent_nearby = distance < self.opponent_threshold

    def process_escape(self):
        """
        Timer callback that executes escape behavior at a fixed rate.
        If either a wall or an opponent is too close, an escape maneuver (reverse and turn) is executed.
        Otherwise, the robot remains stationary.
        """
        twist = Twist()  # Prepare a Twist message for velocity command
        
        # Check if any immediate threat is detected
        if self.near_wall or self.opponent_nearby:
            # Set a reverse movement and a turn to avoid collision
            twist.linear.x = -0.3  # Move backwards
            twist.angular.z = 0.5  # Turn at a fixed rate (adjust for desired behavior)
            self.get_logger().info("Executing escape maneuver due to detected threat.")
        else:
            # No threat detected; stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the command velocity
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    """
    Entry point for the EscapeNode.
    Initializes ROS2, spins the node, and shuts down gracefully.
    """
    rclpy.init(args=args)
    node = EscapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
