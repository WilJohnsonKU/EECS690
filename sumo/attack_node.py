import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class AttackEnemy(Node):
    def __init__(self):
        super().__init__('attack_enemy')
        self.subscription = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 1)
        self.publisher = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.stateSubscription = self.create_subscription(
            String, '/sumo/active_node', self.node_state_callback, 1)
        self.forceChargeSubscription = self.create_subscription(
            Bool, '/sumo/force_charge', self.force_charge_callback, 1)
        self.direction_publisher = self.create_publisher(String, '/sumo/enemy_direction_guess', 1)

        self.bridge = CvBridge()
        self.active = False
        self.force_charge = False

        # Tolerance (in pixels) for determining if the target is significantly off-center.
        self.direction_tolerance = 200

        # Downscale factor (percentage) to reduce processing load.
        self.scale_percent = 50  # Process at 50% of original resolution

    def node_state_callback(self, msg):
        if msg.data == "attack_enemy":
            self.active = True
        else:
            self.active = False

    def force_charge_callback(self, msg):
        self.force_charge = msg.data

    def image_callback(self, msg):
        # Process only when active.
        if not self.active:
            return

        # If force charge is active, bypass vision processing.
        if self.force_charge:
            twist = Twist()
            twist.linear.x = 1.0
            self.publisher.publish(twist)
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error("Failed to convert image: " + str(e))
            return

        # Downscale the image to speed up processing.
        new_width = int(img.shape[1] * self.scale_percent / 100)
        new_height = int(img.shape[0] * self.scale_percent / 100)
        resized_img = cv2.resize(img, (new_width, new_height))

        # Convert the resized image to HSV.
        hsv = cv2.cvtColor(resized_img, cv2.COLOR_BGR2HSV)

        # Create the orange mask using the unchanged detection thresholds.
        lower_orange = np.array([6, 224, 157])
        upper_orange = np.array([25, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Find contours in the mask.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            # Use the same detection threshold.
            if area > 1:
                ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                # Calculate error relative to the center of the resized image.
                error_scaled = center[0] - (new_width // 2)
                # Convert error back to original image scale.
                error = error_scaled * (100 / self.scale_percent)

                # Publish enemy direction guess if error exceeds tolerance.
                if abs(error) > self.direction_tolerance:
                    guess_msg = String()
                    guess_msg.data = "RIGHT" if error > 0 else "LEFT"
                    self.direction_publisher.publish(guess_msg)

                # Set angular velocity based on the error.
                twist.angular.z = -0.005 * error

                # Use the same forward speed as before.
                twist.linear.x = 0.6

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AttackEnemy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
