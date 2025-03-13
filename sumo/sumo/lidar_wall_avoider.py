# Description:
# This node subscribes to LaserScan data from a LiDAR sensor, processes the data to find the
# distance and angle to the nearest wall, and publishes this information as a Float32MultiArray
# on the 'wall_info' topic. This information can be used by other nodes for wall avoidance or
# other navigation purposes.
from rclpy.node import Node
from rclpy.qos import QoSProfile 
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Point 
from std_msgs.msg import Float32MultiArray 

class LidarWallAvoider(Node): # Define a class named LidarWallAvoider that inherits from Node
    def __init__(self): # Define the constructor for the class
        super().__init__('lidar_wall_avoider') # Call the constructor of the parent class (Node) with the node name 'lidar_wall_avoider'
        
        # Subscriber for LiDAR data
        qos_profile = QoSProfile(depth=10) # Create a QoSProfile object with a depth of 10.  This determines how many messages to keep in the history.
        self.lidar_subscriber = self.create_subscription( # Create a subscriber
            LaserScan, # Subscribe to LaserScan messages
            '/scan_raw', # Subscribe to the 'scan' topic
            self.lidar_callback, # Specify the callback function to be called when a message is received
            qos_profile # Use the defined QoS profile
        )
        
        # Publisher for wall distance and angle
        self.wall_info_publisher = self.create_publisher( # Create a publisher
            Float32MultiArray, # Publish Float32MultiArray messages
            'wall_info', # Publish to the 'wall_info' topic
            qos_profile # Use the defined QoS profile
        )
        
        self.get_logger().info('LidarWallAvoider node has been started.') # Log a message to the console

    def lidar_callback(self, msg): # Define the callback function for the LiDAR subscriber
        # Process LiDAR data to find the nearest wall
        min_distance = float('inf') # Initialize the minimum distance to infinity
        angle_to_wall = 0.0 # Initialize the angle to the wall to 0.0
        
        for i, distance in enumerate(msg.ranges): # Iterate over the distances in the LaserScan message
            if distance < min_distance and distance > msg.range_min: # Check if the current distance is less than the minimum distance and greater than the minimum range of the sensor
                min_distance = distance # Update the minimum distance
                angle_to_wall = msg.angle_min + i * msg.angle_increment # Calculate the angle to the wall
        
        # Publish the distance and angle to the nearest wall
        wall_info = Float32MultiArray() # Create a Float32MultiArray message
        wall_info.data = [min_distance, angle_to_wall] # Set the data of the message to the minimum distance and angle to the wall
        self.wall_info_publisher.publish(wall_info) # Publish the message
        
        self.get_logger().info(f'Distance to wall: {min_distance}, Angle to wall: {angle_to_wall}') # Log the distance and angle to the wall