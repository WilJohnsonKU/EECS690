#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Float32MultiArray, String, Bool
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
import numpy as np
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        required_nodes = ['finder_node', 'imu_node', 'attack_enemy', 'centroid_estimator', 'safety_node', 'lidar_data_publisher']  # <-- Replace with actual node names.
        self.wait_for_nodes(required_nodes, timeout_sec=10)
        
        # Subscribe to raw LIDAR data (flat list of [angle, distance, ...])
        self.lidar_subscription = self.create_subscription(
            Float32MultiArray,
            'lidar_data',
            self.lidar_callback,
            qos_profile)
        
        # Subscribe to Centroid data (flat array: [room_center_x, room_center_y, robot_pos_x, robot_pos_y])
        self.centroid_subscription = self.create_subscription(
            Float32MultiArray,
            'centroid_info',
            self.centroid_callback,
            qos_profile)

        # Subscribe to At Center flag
        self.center_flag_subscription = self.create_subscription(
            Bool,
            '/sumo/at_center',
            self.at_center_callback,
            1)

        self.color_subscription = self.create_subscription(
            Bool,
            '/target_color_status',
            self.camera_callback,
            1)

        # DEBUG

        self.state_subscription = self.create_subscription(
            String, '/sumo/active_node', self.node_state_callback, 1)

        self.vel_publisher = self.create_publisher(Twist, '/controller/cmd_vel', 1)

        self.active_node_publisher = self.create_publisher(String, '/sumo/active_node', 1)

        # Latest sensor readings
        self.latest_lidar_min_distance = 99   # Minimum distance from raw lidar_data
        self.latest_centroid_data = None          # Centroid info: [room_center_x, room_center_y, robot_pos_x, robot_pos_y]
        self.target_color_visible = None
        self.at_center = False

        self.at_state = "UNSET"

        # Safety thresholds (tweak these as needed)
        self.safety_distance_threshold = 0.2  # e.g. if any LIDAR range is below this, robot is too close to the wall

        # Fallback behavior timer
        self.create_timer(2.0, self.ensure_safe_mode)

    def wait_for_nodes(self, node_names, timeout_sec=10):
        self.get_logger().info(f"Waiting for nodes: {node_names}")
        start_time = time.time()
        while rclpy.ok():
            # Retrieve discovered nodes as tuples of (node_name, node_namespace)
            discovered = self.get_node_names_and_namespaces()
            discovered_names = [name for name, _ in discovered]
            if all(name in discovered_names for name in node_names):
                self.get_logger().info("All required nodes have loaded.")
                return
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn(f"Timeout waiting for nodes: {node_names}")
                return
            time.sleep(0.1)

    def set_state(self, new_state, force_ignore_dupe=False):
        if (not force_ignore_dupe) and self.at_state == new_state:
            return

        self.at_state = new_state
        
        state_msg = String()
        state_msg.data = new_state
        self.active_node_publisher.publish(state_msg)

    def node_state_callback(self, msg):
        self.get_logger().info(f"State: {self.at_state} | LMIN: {self.latest_lidar_min_distance} | TCV: {self.target_color_visible} | AC: {self.at_center}")

    def lidar_callback(self, msg):
        data = msg.data
        if len(data) % 2 != 0:
            self.get_logger().error("Malformed lidar_data received.")
            return

        num_points = len(data) // 2
        min_distance = float('inf')
        for i in range(num_points):
            distance = data[2*i + 1]
            if distance < min_distance:
                min_distance = distance
        self.latest_lidar_min_distance = min_distance
        self.decide_behavior()

    def at_center_callback(self, msg):
        self.at_center = msg.data

    def centroid_callback(self, msg):
        data = msg.data
        if len(data) != 2:
            self.get_logger().error("Malformed centroid_info received.")
            return
        self.latest_centroid_data = data

    def camera_callback(self, msg):
        self.target_color_visible = msg.data
        self.decide_behavior()

    def decide_behavior(self):
        # Ensure both LIDAR and centroid data have been received.
        if self.latest_lidar_min_distance is None or self.latest_centroid_data is None:
            self.get_logger().warn("Incomplete sensor data. Falling back to Safe Mode.")
            #self.set_state("safety_node", True)
            return

        # Check raw LIDAR: if any obstacle is too close.
        elif (self.latest_lidar_min_distance < self.safety_distance_threshold): #and (not self.target_color_visible):
            self.get_logger().info("LIDAR indicates wall too close. Activating EscapeToCenter.")
            self.set_state("neutral_position", True)
            return
            
        # If a target color is visible, activate attack behavior.
        if self.target_color_visible is True:
            # self.get_logger().info("Target color detected! Activating ChargeAtColor.")
            self.set_state("attack_enemy")
        elif (self.latest_lidar_min_distance > self.safety_distance_threshold*1.25):
            # self.get_logger().info("No immediate threats. Activating FindTargetColor.")
            self.set_state("find_enemy")
        else:
            self.set_state("neutral_position", True)

    def call_service(self, service_client):
        if not service_client.service_is_ready():
            self.get_logger().warn(f"Service {service_client.srv_name} is not available.")
            return
        request = Trigger.Request()
        future = service_client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future: Future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service executed successfully: {response.message}")
            else:
                self.get_logger().warn(f"Service execution failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def ensure_safe_mode(self):
        # If no sensor data is available over a period, trigger safe mode.
        if (self.latest_lidar_min_distance is None and
            self.latest_centroid_data is None and
            self.target_color_visible is None):
            self.get_logger().warn("No sensor data received for 2 seconds. Activating Safe Mode.")
            self.set_state("safety_node")

def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.vel_publisher.publish(Twist())
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
