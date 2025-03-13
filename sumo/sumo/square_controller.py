# Description:
# This node subscribes to the 'wall_info' topic, which provides the distance and angle to the
# nearest wall. If the robot gets too close to a wall (as determined by the
# 'wall_distance_threshold' parameter), the node publishes a Twist message to the 'cmd_vel' topic
# to turn the robot away from the wall. Otherwise, it stops any turning motion.
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray 

class SquareController(Node): # Define a class named SquareController that inherits from Node
    def __init__(self): # Define the constructor for the class
        super().__init__('square_controller') # Call the constructor of the parent class (Node) with the node name 'square_controller'

        self.subscription = self.create_subscription( # Create a subscriber
            Float32MultiArray, # Subscribe to Float32MultiArray messages
            'wall_info', # Subscribe to the 'wall_info' topic
            self.wall_data_callback, # Specify the callback function to be called when a message is received
            10) # Use a queue size of 10
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Create a publisher for Twist messages on the 'cmd_vel' topic with a queue size of 10
        self.declare_parameter('wall_distance_threshold', 0.5) # Declare a parameter named 'wall_distance_threshold' with a default value of 0.5
        self.wall_distance_threshold = self.get_parameter('wall_distance_threshold').get_parameter_value().double_value # Get the value of the 'wall_distance_threshold' parameter as a double
        self.get_logger().info('SquareController node has been started.') # Log a message to the console

    def wall_data_callback(self, msg): # Define the callback function for the 'wall_info' subscriber
        distance_to_wall = msg.data[0] # Extract the distance to the wall from the received message
        angle_to_wall = msg.data[1] # Extract the angle to the wall from the received message

        if distance_to_wall < self.wall_distance_threshold: # Check if the distance to the wall is less than the threshold
            self.get_logger().warn(f"Too close to wall! Distance: {distance_to_wall}, Angle: {angle_to_wall}") # Log a warning message
            twist = Twist() # Create a new Twist message

            # Adjust turning direction based on the angle to the wall
            if angle_to_wall > 0.0: # Check if the angle to the wall is positive
                # Wall is to the left, turn right
                twist.angular.z = -0.5 #was 0.5 Adjust sign and magnitude as needed # Set the angular velocity to turn right
            else: # If the angle to the wall is not positive
                # Wall is to the right, turn left
                twist.angular.z = 0.5  # Adjust sign and magnitude as needed # Set the angular velocity to turn left

            self.publisher_.publish(twist) # Publish the Twist message
        else: # If the distance to the wall is not less than the threshold
            # If not too close to a wall, stop any turning motion
            twist = Twist() # Create a new Twist message
            twist.linear.x = 0.0 # Set the linear velocity to 0.0
            twist.angular.z = 0.0 # Set the angular velocity to 0.0
            self.publisher_.publish(twist) # Publish the Twist message

def main(args=None): # Define the main function
    rclpy.init(args=args) # Initialize the ROS client library
    square_controller = SquareController() # Create an instance of the SquareController class
    rclpy.spin(square_controller) # Spin the node to process callbacks
    square_controller.destroy_node() # Destroy the node
    rclpy.shutdown() # Shutdown the ROS client library

if __name__ == '__main__': # Check if the script is being run directly
    main() # Call the main function