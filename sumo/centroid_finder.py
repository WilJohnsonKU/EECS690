#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np

# --- Your centroid helper functions ---
def polar_to_cartesian(angles, distances):
    x = distances * np.cos(angles)
    y = distances * np.sin(angles)
    return np.stack((x, y), axis=1)

def local_variance_filter(distances, window=5, threshold=0.02):
    padded = np.pad(distances, (window//2, window//2), mode='reflect')
    local_var = np.array([np.var(padded[i:i+window]) for i in range(len(distances))])
    return local_var < threshold

def cluster_outliers(points, max_dist=0.5, min_points=3):
    """
    Simple clustering: find clusters that are not part of a wall.
    """
    clusters = []
    used = np.zeros(len(points), dtype=bool)
    for i, pt in enumerate(points):
        if used[i]:
            continue
        dists = np.linalg.norm(points - pt, axis=1)
        neighbors = np.where(dists < max_dist)[0]
        if len(neighbors) >= min_points:
            clusters.append(points[neighbors])
            used[neighbors] = True
    return clusters

def estimate_room_and_robot(lidar_data):
    data = np.array(lidar_data)
    angles = data[:, 0]
    distances = data[:, 1]

    # Convert polar to Cartesian coordinates
    points = polar_to_cartesian(angles, distances)

    # Optionally, use local variance to separate wall points from others.
    wall_mask = local_variance_filter(distances)
    wall_points = points[wall_mask]
    non_wall_points = points[~wall_mask]

    # wall_points = points

    # Estimate room center from wall points
    room_center = np.nanmean(wall_points, axis=0)

    robot_location = np.nanmean(non_wall_points, axis=0)

    # Clustering for potential robot detection can be added here.
    return room_center, wall_points, non_wall_points, robot_location

# --- CentroidEstimator Node ---
class CentroidEstimator(Node):
    def __init__(self):
        super().__init__('centroid_estimator')
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        # Subscribe to the lidar_data topic (flat array of [angle, distance, ...])
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'lidar_data',
            self.lidar_callback,
            qos_profile)
        # Publisher for centroid information
        self.centroid_publisher = self.create_publisher(
            Float32MultiArray,
            'centroid_info',
            qos_profile)

        # Publisher for enemy information
        self.enemy_location_publisher = self.create_publisher(
            Float32MultiArray,
            '/sumo/enemy_location_guess',
            qos_profile)

    def lidar_callback(self, msg):
        data = msg.data
        if len(data) % 2 != 0:
            self.get_logger().error("Malformed lidar_data received.")
            return
        num_points = len(data) // 2
        lidar_data = []
        # Reconstruct (angle, distance) pairs
        for i in range(num_points):
            angle = data[2 * i]
            distance = data[2 * i + 1]
            lidar_data.append((angle, distance))
        
        # Run centroid and robot estimation
        room_center, wall_points, non_wall_points, robot_pos = estimate_room_and_robot(lidar_data)
        
        # Prepare output message.
        centroid_msg = Float32MultiArray()
        enemy_location_msg = Float32MultiArray()

        if robot_pos is not None:
            enemy_location_msg.data = [robot_pos[0], robot_pos[1]]
        
        centroid_msg.data = [room_center[0], room_center[1]]
           
        self.centroid_publisher.publish(centroid_msg)
        self.enemy_location_publisher.publish(enemy_location_msg)

        # self.get_logger().info(f"Enemy Direction Guess: { robot_pos[0] }, { robot_pos[1] }")

def main(args=None):
    rclpy.init(args=args)
    node = CentroidEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
