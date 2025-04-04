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

        # Subscribers
        self.subscription = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 1)
        self.state_subscription = self.create_subscription(
            String, '/sumo/active_node', self.node_state_callback, 1)
        self.enemy_direction_subscriber = self.create_subscription(
            String, '/sumo/enemy_direction_guess', self.enemy_direction_callback, 1)

        # Publishers
        self.color_publisher = self.create_publisher(Bool, '/target_color_status', 1)
        self.force_charge_publisher = self.create_publisher(Bool, '/sumo/force_charge', 1)
        self.publisher = self.create_publisher(Twist, '/controller/cmd_vel', 1)

        # OpenCV Bridge
        self.bridge = CvBridge()

        # State variables
        self.active = False
        self.enemy_direction = "UNKNOWN"
        self.force_charge = False

        # Processing parameters (tweak these based on your needs)
        self.scale_percent = 50  # Downscale ROI to 50% of original size
        self.orange_threshold = 1  # Minimum non-zero count for orange detection
        self.enemy_threshold = 30000  # Expected enemy area in full-res pixels

        self.current_twist_z = 0

        self.get_logger().info("FinderNode started. Waiting for activation...")

    def node_state_callback(self, msg):
        if msg.data == "find_enemy":
            self.active = True
        else:
            self.active = False

    def enemy_direction_callback(self, msg):
        direction_guess = msg.data.strip()
        if direction_guess in ["LEFT", "RIGHT"]:
            self.enemy_direction = direction_guess

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        # Early crop: use only the bottom part of the image (from ~33% to the bottom)
        height, width = img.shape[:2]
        roi = img[int(height / 4):, :]

        # Downscale ROI to reduce processing load
        new_width = int(roi.shape[1] * self.scale_percent / 100)
        new_height = int(roi.shape[0] * self.scale_percent / 100)
        roi = cv2.resize(roi, (new_width, new_height))

        # Convert the downscaled ROI to HSV once
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- Orange Detection ---
        # Define HSV range for bright neon orange
        lower_orange = np.array([6, 224, 157])
        upper_orange = np.array([25, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # Use countNonZero instead of contours for faster area estimation
        orange_area = cv2.countNonZero(mask_orange)
        # (Optionally, you could log this value occasionally for debugging)
        # self.get_logger().warn(f"Orange AREA (scaled): {orange_area}")

        # --- Enemy (Black) Detection ---
        enemy_found = self.check_for_enemy(hsv)

        # Adjusted behavior based on detection
        if orange_area > self.orange_threshold:
            # Reset force charge if it was set
            if self.force_charge:
                self.force_charge = False
                msg_fc = Bool()
                msg_fc.data = False
                self.force_charge_publisher.publish(msg_fc)

            # Optionally log the orange detection area (for debug purposes)
            self.get_logger().warn(f"Detected orange area: {orange_area}")

            if self.active:
                twist = Twist()
                twist.angular.z = self.current_twist_z * -1.0
                #self.publisher.publish(twist)
            self.active = False
            msg_color = Bool()
            msg_color.data = True
            self.color_publisher.publish(msg_color)

        elif enemy_found:
            self.force_charge = True
            msg_fc = Bool()
            msg_fc.data = True
            self.force_charge_publisher.publish(msg_fc)

        else:
            # If enemy detection is no longer valid, reset force charge
            if self.force_charge:
                self.force_charge = False
                msg_fc = Bool()
                msg_fc.data = False
                self.force_charge_publisher.publish(msg_fc)

            msg_color = Bool()
            msg_color.data = False
            self.color_publisher.publish(msg_color)

            if self.active:
                self.rotate_in_search()

    def rotate_in_search(self):
        twist = Twist()
        if not self.enemy_direction or self.enemy_direction.upper() == "UNKNOWN":
            self.enemy_direction = random.choice(["LEFT", "RIGHT"])
            self.get_logger().info("No valid enemy direction provided. Using random direction: " + self.enemy_direction)
        if self.enemy_direction == "LEFT":
            twist.angular.z = 1.5
        elif self.enemy_direction == "RIGHT":
            twist.angular.z = -1.5

        self.current_twist_z = twist.angular.z
        self.publisher.publish(twist)

    def check_for_enemy(self, hsv):
        """Detects the enemy car based on a large black area using connected components."""
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)
        
        # Use connectedComponentsWithStats for efficient blob analysis
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(black_mask)
        if num_labels > 1:
            # Find the area of the largest connected component (ignoring background label 0)
            max_area = np.max(stats[1:, cv2.CC_STAT_AREA])
            # Adjust threshold for downscaled image: (scale_percent/100)^2 factor
            adjusted_threshold = self.enemy_threshold * (self.scale_percent / 100) ** 2
            if max_area > adjusted_threshold:
                return True
        return False

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
