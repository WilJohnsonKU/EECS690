import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import time

class MoonwalkNode(Node):
    def __init__(self):
        super().__init__('moonwalk_node')
        
        # Create publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters for authentic moonwalk
        self.forward_speed = 0.15    # Speed for forward step
        self.backward_speed = -0.1   # Speed for backward glide
        self.side_speed = 0.05       # Speed for sideways movement
        self.rotation_speed = 0.1    # Speed for slight rotation
        
        # Timing parameters
        self.step_duration = 0.5     # Duration of forward step
        self.glide_duration = 1.0    # Duration of backward glide

        self.create_service(Trigger, 'trigger_dance', self.moonwalk_callback)
        
        self.get_logger().info('Moonwalk node has been started')

    def moonwalk_callback(self, request, response):
        # Create a bark pattern (two-tone bark)
        self.moonwalk_pattern()
        response.success = True
        return response

    def moonwalk_pattern(self):
        """Execute the moonwalk movement pattern"""
        # Create movement message
        twist = Twist()
        
        # Step 1: Quick forward step with slight side movement
        twist.linear.x = self.forward_speed
        twist.linear.y = self.side_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.step_duration)
        
        # Step 2: Smooth backward glide with rotation
        twist.linear.x = self.backward_speed
        twist.linear.y = -self.side_speed
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.glide_duration)
        
        # Step 3: Rotate and move to the side 
        twist.linear.y = self.side_speed
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.step_duration)

        # Step 4: Quick forward step with slight side movement
        twist.linear.x = self.forward_speed
        twist.linear.y = -self.side_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.step_duration)

        # Step 5: Backwards glide with reverse rotation
        twist.linear.x = self.backward_speed
        twist.linear.y = self.side_speed
        twist.angular.z = -self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.step_duration)

        # Step 6: Rotate and move to the side
        twist.linear.y = -self.side_speed
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.step_duration)

        # Step 7: Quick forward step with slight side movement
        twist.linear.x = self.forward_speed
        twist.linear.y = self.side_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.step_duration)
        
        # Step 8: Smooth backward glide with rotation
        twist.linear.x = self.backward_speed
        twist.linear.y = -self.side_speed
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.glide_duration)

        # Stop the robot
        self.stop_robot()

    def stop_robot(self):
        """Stop all robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    moonwalk_node = MoonwalkNode()
    
    try:
        rclpy.spin(moonwalk_node)
    except KeyboardInterrupt:
        # Stop the robot on shutdown
        moonwalk_node.stop_robot()
        
    moonwalk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()