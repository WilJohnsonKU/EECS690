import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BuzzerState
from std_srvs.srv import Trigger
import time

class BarkNode(Node):
    def __init__(self):
        super().__init__('bark_node')
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        self.create_service(SetBark, 'trigger_bark', self.bark_callback)
        self.get_logger().info('Bark node has been started')

    def bark_callback(self, request, response):
        # Create a bark pattern (two-tone bark)
        self.bark_pattern()
        response.success = True
        return response

    def bark_pattern(self):
        buzzer_msg = BuzzerState()
        
        # First part of bark (higher pitch)
        buzzer_msg.freq = 800  # Higher frequency
        buzzer_msg.on_time = 0.15
        buzzer_msg.off_time = 0.05
        buzzer_msg.repeat = 1
        self.buzzer_pub.publish(buzzer_msg)
        time.sleep(0.2)

        # Second part of bark (lower pitch)
        buzzer_msg.freq = 600  # Lower frequency
        buzzer_msg.on_time = 0.2
        buzzer_msg.off_time = 0.05
        buzzer_msg.repeat = 1
        self.buzzer_pub.publish(buzzer_msg)

def main(args=None):
    rclpy.init(args=args)
    bark_node = BarkNode()
    rclpy.spin(bark_node)
    bark_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
