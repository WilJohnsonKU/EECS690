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
        #cv2.namedWindow("Finder Visualizer", cv2.WINDOW_NORMAL)
        #cv2.waitKey(1)

        self.get_logger().info("FinderNode started. Waiting for activation...")

    def node_state_callback(self, msg):
        """Handles activation/deactivation from external control."""
        if msg.data == "find_enemy":
            if self.active == False:
                self.active = True
        else:
            if self.active == True:
                self.active = False

    def image_callback(self, msg):
        """Processes camera frames to detect the color orange."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        # Convert image to HSV and create orange mask
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Height Mask
        # Get image dimensions
        height = hsv.shape[0]

        # Calculate starting row (25% from top)
        start_row = height // 4

        # Crop the image from 25% height to the bottom
        hsv = hsv[start_row:, :]

        # Define an HSV range tuned for a bright neon orange.
        # Adjust the lower saturation/value if lighting is less intense.
        lower_orange = np.array([6, 224, 157])
        upper_orange = np.array([25, 255, 255])

        """upper_orange1 = np.array([25, 255, 255])
        lower_orange1 = np.array([10, 100, 100])
        lower_orange2 = np.array([5, 100, 100])
        upper_orange2 = np.array([10, 255, 255])"""

        # Create the mask for the defined range
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        #mask1 = cv2.inRange(hsv, lower_orange1, upper_orange1)
        #mask2 = cv2.inRange(hsv, lower_orange2, upper_orange2)
        #mask = cv2.bitwise_or(mask1, mask2)

        # Apply a small morphological opening to remove noise
        # kernel = np.ones((3, 3), np.uint8)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

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
        else:
            msg = Bool()
            msg.data = False
            self.color_publisher.publish(msg)
            
            if self.active:
                self.rotate_in_search()

        self.check_for_enemy(hsv)

        # Display the processed image (optional)
        # Convert cropped HSV back to BGR for displaying
        #bgr_cropped = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        # Show it in a window
         # (Optional) Draw contours on the original image for visualization:
        #cv2.drawContours(bgr_cropped, contours, -1, (0, 255, 0), 2)
        #cv2.imshow("Finder Visualizer", bgr_cropped)
        #cv2.waitKey(1)

    def rotate_in_search(self):
        """Rotates the robot in a chosen direction until orange is detected."""
        twist = Twist()
        twist.angular.z = 3.0 * self.search_direction  # Rotate in the chosen direction
        self.publisher.publish(twist)

    def check_for_enemy(self, hsv):
        """Detects the enemy car based on black color.

        Uses a defined HSV range to isolate black regions, applies morphological 
        operations to clean up noise, and then filters detected regions by contour area.
        """
        # Define HSV range for black
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        # Create a mask for black areas
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        # Apply morphological operations to reduce noise
        """kernel = np.ones((3, 3), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)"""

        # Find contours from the black mask
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        enemy_detected = False
        enemy_threshold = 15000  # Minimum area to consider a detection as the enemy car
        
        for cnt in contours:
            if cv2.contourArea(cnt) > enemy_threshold:
                enemy_detected = True
                self.get_logger().info("[Black] Enemy car detected.")
                break

        enemy_pixel_count = cv2.countNonZero(black_mask)

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
