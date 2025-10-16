import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/object_detection', 10)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10
        )
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to HSV color space
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define HSV range for green color
            lower_green = np.array([30, 80, 80]) #35/100/100
            upper_green = np.array([90, 255, 255])
            
            # Create a mask that isolates the green pixels
            mask = cv2.inRange(hsv_image, lower_green, upper_green)
            
            # Define a kernel (e.g., a 5x5 ellipse)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
            
            # This fills small holes (like the stopper's center) in the mask.
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 4. Find contours (i.e., continuous boundaries of the shapes) in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:  # Filter small objects
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = h / float(w)
                    # Typical bottle aspect ratio filter (adjust as needed)
                    if 1.5 < aspect_ratio < 4.0:
                        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
            
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            # Publish the processed image
            self.publisher.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()