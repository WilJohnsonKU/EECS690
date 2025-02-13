#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        # Publisher for incoming serial data (if needed for debugging or other purposes)
        self.serial_in_pub = self.create_publisher(String, 'serial_in', 10)
        # Subscriber for outgoing data to serial (this part remains unchanged)
        self.serial_out_sub = self.create_subscription(String, 'serial_out', self.serial_out_callback, 10)
        # Publisher for robot movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Buffer to accumulate serial data until a complete packet is received
        self.buffer = b""

        # Open the serial port. (Using 'rb+' opens it for both reading and writing.)
        try:
            self.serial_port = open('/dev/rfcomm0', 'rb+', buffering=0)
            # Set non-blocking mode so that read() returns immediately.
            os.set_blocking(self.serial_port.fileno(), False)
            self.get_logger().info("Opened /dev/rfcomm0 successfully in non-blocking mode")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # Set up a timer to check for incoming data every 0.1 seconds.
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        """
        Reads incoming serial data and accumulates it in a buffer.
        Processes complete packets that are terminated by a newline ('\n').
        """
        try:
            # Try to read up to 64 bytes from the serial port.
            data = self.serial_port.read(64)
            if data:
                self.buffer += data
                # Process complete packets delimited by newline (\n)
                while b'\n' in self.buffer:
                    # Split at the first newline
                    packet, self.buffer = self.buffer.split(b'\n', 1)
                    # Decode the packet and remove any surrounding whitespace (including carriage returns)
                    command_str = packet.decode('utf-8', errors='ignore').strip()
                    if command_str:
                        self.get_logger().info(f"Received packet: {command_str}")
                        # Process the command to move the robot as needed.
                        self.process_command(command_str)
                        # Optionally, publish the received command on the serial_in topic.
                        msg = String()
                        msg.data = command_str
                        self.serial_in_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")

    def process_command(self, command: str):
        """
        Processes the received command and publishes an appropriate Twist message.
        Commands:
            - MOVE_FORWARD: moves the robot forward
            - MOVE_BACKWARDS: moves the robot backward
            - STOP: stops the robot
        """
        twist = Twist()
        if command == "MOVE_FORWARD":
            twist.linear.x = 0.5    # Adjust speed as necessary.
            twist.angular.z = 0.0
            self.get_logger().info("Command: MOVE_FORWARD. Moving forward.")
        elif command == "MOVE_BACKWARDS":
            twist.linear.x = -0.5   # Adjust speed as necessary.
            twist.angular.z = 0.0
            self.get_logger().info("Command: MOVE_BACKWARDS. Moving backwards.")
        elif command == "STOP":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Command: STOP. Stopping robot.")
        else:
            self.get_logger().warning(f"Unknown command received: {command}")
            return  # Ignore unknown commands

        # Publish the Twist message to control the robot.
        self.cmd_vel_pub.publish(twist)

    def serial_out_callback(self, msg):
        """
        Sends outgoing data to the serial port.
        """
        try:
            out_data = msg.data.encode('utf-8')
            self.get_logger().info(f"Attempting to send: {out_data} to serial...")
            self.serial_port.write(out_data)
            self.get_logger().info(f"Sent to serial: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
