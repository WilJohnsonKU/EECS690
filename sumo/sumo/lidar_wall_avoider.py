#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math

class LidarDataPublisher(Node):
    def __init__(self):
        super().__init__('lidar_data_publisher')
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.lidar_callback,
            qos_profile)
        self.lidar_publisher = self.create_publisher(
            Float32MultiArray,
            'lidar_data',
            qos_profile)
        
        # Use a fixed number of bins for a consistent resolution.
        self.num_bins = 360  # maximum reasonable number of points
        self.world_model = None  # list of length self.num_bins holding the best-guess ranges
        self.model_angle_min = None
        self.model_angle_max = None

    def lidar_callback(self, msg):
        if msg is None:
            self.get_logger().warn("No LIDAR Data Received")
            return

        # On the first scan, initialize the persistent world model using the scan's angular span.
        if self.world_model is None:
            self.model_angle_min = msg.angle_min
            self.model_angle_max = msg.angle_max
            self.world_model = [float('nan')] * self.num_bins

        # Compute the angular span and bin resolution.
        angle_range = self.model_angle_max - self.model_angle_min
        bin_resolution = angle_range / (self.num_bins - 1)

        # Iterate through the new scan measurements.
        current_angle = msg.angle_min
        for r in msg.ranges:
            # Only consider measurements that fall within our persistent model range.
            if current_angle < self.model_angle_min or current_angle > self.model_angle_max:
                current_angle += msg.angle_increment
                continue

            # Map the current angle to a bin index.
            # We compute a fractional index and round to the nearest integer.
            fractional_index = (current_angle - self.model_angle_min) / angle_range * (self.num_bins - 1)
            bin_index = int(round(fractional_index))

            # Update the persistent world model only if the new measurement is valid.
            if not math.isnan(r):
                self.world_model[bin_index] = r
            # Otherwise, leave the previous value intact.
            current_angle += msg.angle_increment

        # Build a flat list of angle–range pairs from the persistent world model.
        flat_data = []
        for i in range(self.num_bins):
            angle = self.model_angle_min + i * bin_resolution
            distance = self.world_model[i]
            flat_data.extend([angle, distance])

        lidar_array = Float32MultiArray()
        # Use slicing to copy the list.
        lidar_array.data = flat_data[:]
        self.lidar_publisher.publish(lidar_array)
        # self.get_logger().info(f"{lidar_array}\n\n\n")
        # self.get_logger().info("Published world model with {} bins.".format(self.num_bins))

def main(args=None):
    rclpy.init(args=args)
    node = LidarDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
