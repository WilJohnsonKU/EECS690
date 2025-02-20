import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
import numpy as np

class RedBallFetcher(Node):
    def __init__(self):
        super().__init__('red_ball_fetcher')
        # Subscribe to the MentorPi camera topic.
        self.subscription = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.active = False

        # Create a trigger service to start processing images.
        self.create_service(Trigger, 'trigger_fetch', self.fetch_callback)
        self.get_logger().info("Red Ball Fetcher node started.")

        # Create an OpenCV window so that images can be displayed.
        cv2.namedWindow("Fetch", cv2.WINDOW_NORMAL)

    def fetch_callback(self, request, response):
        self.active = True
        self.get_logger().info("Fetch triggered: now processing images for red ball detection.")
        response.success = True
        response.message = "Fetch started."
        return response

    def image_callback(self, msg):
        # Only process images if fetch has been triggered.
        if not self.active:
            return

        self.get_logger().debug("Image received for processing.")
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error("Failed to convert image: " + str(e))
            return

        # Convert image to HSV and create red masks.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the mask.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            if area > 500:
                ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                cv2.circle(img, center, int(radius), (0, 255, 0), 2)

                # Calculate error relative to the image center.
                error = center[0] - (img.shape[1] // 2)
                twist.angular.z = -0.005 * error

                # Move forward if the ball is still far away.
                if radius < 50:  # adjust threshold if needed
                    twist.linear.x = 0.2
                    self.get_logger().debug(f"Moving forward: radius={radius:.2f}, error={error}")
                else:
                    # Stop if the ball is close enough.
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.active = False
                    self.get_logger().info("Red ball reached. Stopping.")
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)
        cv2.imshow("Fetch", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RedBallFetcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
