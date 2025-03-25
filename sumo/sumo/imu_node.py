import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.subscription = self.create_subscription(
            Imu, 'sensor_msgs/msg/Imu', self.imu_callback, 10)
        self.get_logger().info("IMU Node started and subscribed to sensor_msgs/msg/Imu")

    def imu_callback(self, msg):
        self.get_logger().info(f"IMU Data:\n"
                               f"Header: {msg.header}\n"
                               f"Orientation: {msg.orientation}\n"
                               f"Orientation Covariance: {msg.orientation_covariance}\n"
                               f"Angular Velocity: {msg.angular_velocity}\n"
                               f"Angular Velocity Covariance: {msg.angular_velocity_covariance}\n"
                               f"Linear Acceleration: {msg.linear_acceleration}\n"
                               f"Linear Acceleration Covariance: {msg.linear_acceleration_covariance}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()