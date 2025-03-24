import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
import numpy as np

class FinderNode(Node):
    def __init__(self):
        super().__init__('finder_node')

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 10)

        # Subscribe to node state topic to check if this node is active
        self.state_subscription = self.create_subscription(
            String, '/sumo/active_node', self.node_state_callback, 10)

        self.color_publisher = self.create_publisher(
            Bool, '/target_color_status', 10
        )

        # Publisher for robot motion
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        # State variables
        self.active = False
        self.search_direction = random.choice([-1, 1])  # This should be based on LIDAR's best guess as the where the oponent is

        
        # Create an OpenCV window so that images can be displayed.
        cv2.namedWindow("Finder Visualizer", cv2.WINDOW_NORMAL)

        self.get_logger().info("FinderNode started. Waiting for activation...")

    def node_state_callback(self, msg):
        """Handles activation/deactivation from external control."""
        if msg.data == "find_enemy":
            if not self.active:
                self.active = True
        else:
            if self.active:
                self.active = False

    def image_callback(self, msg):
        """Processes camera frames to detect the color orange."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return
        cv2.imshow("Finder Visualizer", img)

        # Convert image to HSV and create orange mask
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define an HSV range tuned for a bright neon orange.
        # Adjust the lower saturation/value if lighting is less intense.
        lower_orange = np.array([5, 50, 50])
        upper_orange = np.array([25, 255, 255])

        # Create the mask for the defined range
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Apply a small morphological opening to remove noise
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours in the mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            # self.get_logger().info(f"Color Area for contour: {area}")
            if area > 10:
                self.active = False
                msg = Bool()
                msg.data = True
                self.color_publisher.publish(msg)

            # (Optional) Draw contours on the original image for visualization:
            # cv2.drawContours(img, contours, -1, (0, 255, 0), 2)
        else:
            msg = Bool()
            msg.data = False
            self.color_publisher.publish(msg)
            
            if self.active:
                self.rotate_in_search()

        # Display the processed image (optional)
        # cv2.waitKey(1)

    def rotate_in_search(self):
        """Rotates the robot in a chosen direction until orange is detected."""
        twist = Twist()
        twist.angular.z = 3.0 * self.search_direction  # Rotate in the chosen direction
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FinderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
