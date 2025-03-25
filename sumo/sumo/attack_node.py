import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
import numpy as np

class AttackEnemy(Node):
    def __init__(self):
        super().__init__('attack_enemy')
        # Subscribe to the MentorPi camera topic.
        self.subscription = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.stateSubscription = self.create_subscription(
            String, '/sumo/active_node', self.node_state_callback, 10
        )
        self.bridge = CvBridge()
        self.active = False

    def node_state_callback(self, msg):
        if (msg.data == "attack_enemy"):
            self.active = True
        else:
            self.active = False
        return

    def image_callback(self, msg):
        # Only process images if fetch has been triggered.
        if not self.active:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error("Failed to convert image: " + str(e))
            return

        # Convert image to HSV and create orange masks.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([6, 224, 157])
        upper_orange = np.array([25, 255, 255])

        """upper_orange1 = np.array([25, 255, 255])
        lower_orange1 = np.array([10, 100, 100])
        lower_orange2 = np.array([5, 100, 100])
        upper_orange2 = np.array([10, 255, 255])"""

        # Create the mask for the defined range
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Find contours in the mask.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            # self.get_logger().info(f"Color Area for contour: {area}")
            if area > 1:
                ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))

                # Calculate error relative to the image center (x-axis).
                error = center[0] - (img.shape[1] // 2)

                # Angular velocity for turning toward the center.
                twist.angular.z = -0.005 * error

                # Normalize error to range [0, 1], where 0 = perfectly centered.
                max_error = img.shape[1] // 2
                centeredness = 1.0 - min(abs(error) / max_error, 1.0)

                # Set forward speed based on how centered the object is.
                # More centered => faster forward speed
                # Tune min/max speeds as needed
                min_speed = 0.2
                max_speed = 0.6
                twist.linear.x = min_speed + (max_speed - min_speed) * centeredness

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
