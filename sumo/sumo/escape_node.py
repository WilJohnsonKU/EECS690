import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class EscapeNode(Node):
    def __init__(self):
        super().__init__('escape_node')

        # Publisher to command the robot's movement.
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber for LiDAR data to detect walls.
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        
        # Subscriber for opponent information. This topic is assumed to publish [distance, angle] data.
        self.opponent_sub = self.create_subscription(Float32MultiArray, 'opponent_info', self.opponent_callback, 10)

        # Internal state variables to track proximity status.
        self.near_wall = False
        self.opponent_nearby = False

        # Thresholds for triggering wall avoidance and opponent escape.
        self.wall_threshold = 0.4  # Distance (meters) at which wall avoidance is triggered.
        self.opponent_threshold = 0.7  # Distance (meters) at which opponent escape is triggered.

        self.get_logger().info("EscapeNode initialized.")

    def lidar_callback(self, msg):
        """
        Callback function for processing incoming LiDAR data.
        It checks for walls in front, left, and right, then triggers the corresponding behavior.
        """
        # Get the minimum distance from a subset of LiDAR readings for the front.
        front = min(msg.ranges[0:10] + msg.ranges[-10:])
        # Example slices for left and right - these might need adjusting depending on your LiDAR's configuration.
        left = min(msg.ranges[60:100])
        right = min(msg.ranges[-100:-60])

        # Check if any wall is within the threshold distance.
        if front < self.wall_threshold or left < self.wall_threshold or right < self.wall_threshold:
            self.near_wall = True
            self.avoid_wall(front, left, right)
        else:
            self.near_wall = False
            # If there is no wall nearby but an opponent is detected, initiate escape.
            if self.opponent_nearby:
                self.escape_opponent()
            else:
                self.idle_behavior()

    def opponent_callback(self, msg):
        """
        Callback function for processing opponent information.
        It expects msg.data to be a list with the distance as the first element.
        """
        # Assume the first element of msg.data is the distance to the opponent.
        distance = msg.data[0]
        # Set opponent_nearby to True if the opponent is within the defined threshold.
        self.opponent_nearby = distance < self.opponent_threshold

    def avoid_wall(self, front, left, right):
        """
        Executes wall avoidance maneuvers by publishing a Twist message.
        Prioritizes turning away from the wall.
        """
        twist = Twist()
        twist.linear.x = 0.0  # Stop any forward motion to prevent collision.
        
        # Determine turning direction based on which side is too close.
        if front < self.wall_threshold:
            twist.angular.z = 0.6  # If wall is directly ahead, turn right.
        elif left < self.wall_threshold:
            twist.angular.z = -0.6  # If wall is on the left, turn left.
        elif right < self.wall_threshold:
            twist.angular.z = 0.6  # If wall is on the right, turn right.
        
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Wall too close! Executing wall avoidance.")

    def escape_opponent(self):
        """
        Executes escape maneuvers from an opponent.
        This method reverses slightly and turns to avoid the opponent.
        """
        twist = Twist()
        twist.linear.x = -0.2  # Move backward to create space.
        twist.angular.z = 0.5  # Turn to help change the orientation.
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Opponent nearby! Escaping.")

    def idle_behavior(self):
        """
        Sets the robot to an idle state when no threat (wall or opponent) is detected.
        """
        twist = Twist()
        twist.linear.x = 0.0  # Stop forward/backward movement.
        twist.angular.z = 0.0  # Stop any rotation.
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Idle - Safe zone.")

def main(args=None):
    rclpy.init(args=args)
    node = EscapeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
