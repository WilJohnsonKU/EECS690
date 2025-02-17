import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedBallFetcher(Node):
    def __init__(self):
        super().__init__('red_ball_fetcher')
        self.subscription = self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error("Conversion error: " + str(e))
            return
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) > 500:
                ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                center = (int(x), (int(y)))
                cv2.circle(img, center, int(radius), (0, 255, 0), 2)
                error = center[0] - img.shape[1] // 2
                twist.angular.z = -0.002 * error
                twist.linear.x = 0.2 if abs(error) < 20 else 0.0
        self.publisher.publish(twist)
        cv2.imshow("Red Ball Fetcher", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RedBallFetcher()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
