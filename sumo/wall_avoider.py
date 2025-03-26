# Description:
# This node subscribes to LaserScan data from a LiDAR sensor and processes it to avoid walls.
# It calculates the distances to walls in front, left, and right, and then publishes Twist
# messages to the 'cmd_vel' topic to control the robot's motion, ensuring it avoids obstacles.
import rclpy 
from rclpy.node import Node
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan

MAX_SCAN_ANGLE = 240

class WallAvoider(Node): # Define a class named WallAvoider that inherits from Node
    def __init__(self): # Define the constructor for the class
        super().__init__('wall_avoider') # Call the constructor of the parent class (Node) with the node name 'wall_avoider'
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_subscriber = self.create_subscription( # Create a subscriber
            LaserScan, # Subscribe to LaserScan messages
            '/scan_raw', # Subscribe to the 'scan' topic
            self.lidar_callback, # Specify the callback function to be called when a message is received
            qos # Use the qos_profile_sensor_data QoS profile
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1) # Create a publisher for Twist messages on the 'cmd_vel' topic with the default QoS profile
        self.twist = Twist() # Create a Twist message object

    def lidar_callback(self, msg): # Define the callback function for the LiDAR subscriber
        # Process LiDAR data to find the distance to the nearest wall
        print("\n\n")
        print(msg.ranges)
        front_distance = min(msg.ranges[0:10] + msg.ranges[-10:])  # Front distance: Calculate the minimum distance from the front lidar readings
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / msg.angle_increment)
        left_distance = min(msg.ranges[:max_index])
        right_distance = min(msg.ranges[::-1][:max_index])

        self.avoid_walls(front_distance, left_distance, right_distance) # Call the avoid_walls method with the calculated distances

    def avoid_walls(self, front_distance, left_distance, right_distance): # Define the avoid_walls method
        # Define a threshold distance to walls
        threshold = 0.5  # 50 cm: Define the threshold distance to the wall

        if front_distance < threshold: # Check if the front distance is less than the threshold
            # If too close to the wall in front, turn
            self.twist.linear.x = 0.0 # Set the linear velocity to 0.0 to stop forward motion
            self.twist.angular.z = 0.5  # Turn right: Set the angular velocity to turn right
        elif left_distance < threshold: # Check if the left distance is less than the threshold
            # If too close to the wall on the left, turn right
            self.twist.linear.x = 0.0 # Set the linear velocity to 0.0 to stop forward motion
            self.twist.angular.z = -0.5  # Turn left: Set the angular velocity to turn left
        elif right_distance < threshold: # Check if the right distance is less than the threshold
            # If too close to the wall on the right, turn left
            self.twist.linear.x = 0.0 # Set the linear velocity to 0.0 to stop forward motion
            self.twist.angular.z = 0.5  # Turn right: Set the angular velocity to turn right
        else: # If no walls are detected
            # Move forward if no walls are detected
            self.twist.linear.x = 0.2  # Move forward: Set the linear velocity to move forward
            self.twist.angular.z = 0.0 # Set the angular velocity to 0.0 to stop turning

        self.cmd_vel_publisher.publish(self.twist) # Publish the Twist message

def main(args=None): # Define the main function
    rclpy.init(args=args) # Initialize the ROS client library
    wall_avoider = WallAvoider() # Create an instance of the WallAvoider class
    rclpy.spin(wall_avoider) # Spin the node to process callbacks
    wall_avoider.destroy_node() # Destroy the node
    rclpy.shutdown() # Shutdown the ROS client library

if __name__ == '__main__': # Check if the script is being run directly
    main() # Call the main function