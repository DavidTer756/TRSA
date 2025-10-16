import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter_node')

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, '/camera/image_processed', 10)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_rect', 
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert BGR to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian smoothing
            smoothed_image = cv2.GaussianBlur(gray_image, (5, 5), 0) 
            
            # Optional: Canny edge detection
            edges_image = cv2.Canny(smoothed_image, 50, 150) 
            
            # Convert back to ROS Image message
            gray_msg = self.bridge.cv2_to_imgmsg(edges_image, encoding='mono8')
            
            # Publish the processed image
            self.publisher.publish(gray_msg)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
