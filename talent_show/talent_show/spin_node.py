#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import time

class SpinNode(Node):
    def __init__(self):
        super().__init__('spin_node')
        
        # Publisher for robot movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create the spin trigger service
        self.create_service(Trigger, 'trigger_spin', self.spin_callback)
        self.get_logger().info('Spin node has been started')

        # Setup client to self-trigger spin for testing
        self.client = self.create_client(Trigger, 'trigger_spin')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = Trigger.Request()

    def spin_callback(self, request, response):
        """
        Callback for the 'trigger_spin' service.
        Executes one of two spin behaviors:
        - In-place spin
        - Circular drive (moving while spinning)
        """
        self.get_logger().info("Spin command received! Choosing spin mode...")

        # Choose between the two spin modes randomly or based on some condition
        spin_mode = self.choose_spin_mode()

        if spin_mode == "in_place":
            self.spin_in_place()
        elif spin_mode == "circle":
            self.drive_in_circle()
        
        response.success = True
        return response

    def choose_spin_mode(self):
        """
        Determines whether the robot spins in place or drives in a circle.
        Change logic here if you want a specific trigger-based decision.
        """
        # Example: Random choice (Uncomment below if you want random behavior)
        # import random
        # return random.choice(["in_place", "circle"])
        
        # Defaulting to a simple alternating behavior (modify as needed)
        return "in_place" if int(time.time()) % 2 == 0 else "circle"

    def spin_in_place(self):
        """
        Spins the robot in place for 3 seconds.
        """
        self.get_logger().info("Spinning in place!")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 3.0  # Adjust speed as necessary

        self.cmd_vel_pub.publish(twist)
        time.sleep(3)

        # Stop the robot after spinning
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Spin complete.")

    def drive_in_circle(self):
        """
        Moves the robot in a circular trajectory for 3 seconds.
        """
        self.get_logger().info("Driving in a circle!")
        twist = Twist()
        twist.linear.x = 0.5  # Adjust forward speed
        twist.angular.z = 1.5  # Adjust turning speed for a circular motion

        self.cmd_vel_pub.publish(twist)
        time.sleep(3)

        # Stop the robot after moving in a circle
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Circle drive complete.")

def main(args=None):
    rclpy.init(args=args)
    spin_node = SpinNode()
    # Test trigger (uncomment to auto-trigger the spin when running)
    # spin_node.send_request()
    rclpy.spin(spin_node)
    spin_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

