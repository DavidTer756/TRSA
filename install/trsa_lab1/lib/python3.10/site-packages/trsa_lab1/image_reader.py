import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DualImageViewer(Node):
    def __init__(self):
        super().__init__('dual_image_viewer_node')

        self.bridge = CvBridge()

        self.raw_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.raw_image_callback,  
            10)
        
        self.rectified_subscription = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.rectified_image_callback,  
            10)
        
        self.convert_subscription = self.create_subscription(
            Image,
            '/camera/image_processed',
            self.converter_image_callback,  
            10)
        
        self.convert_subscription = self.create_subscription(
            Image,
            '/camera/object_detection',
            self.detetion_image_callback,  
            10)

    def raw_image_callback(self, msg):
        raw_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Imagem Original (Raw)", raw_frame)
        cv2.waitKey(1)
        
    def rectified_image_callback(self, msg):
        rectified_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Imagem Retificada (Rect)", rectified_frame)
        cv2.waitKey(1)
        
    def converter_image_callback(self, msg):
        convert_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Imagem Convertida (Convert)", convert_frame)
        cv2.waitKey(1)
        
    def detetion_image_callback(self, msg):
        convert_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Detencao de objeto (detect)", convert_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    dual_viewer_node = DualImageViewer()
    rclpy.spin(dual_viewer_node)
    dual_viewer_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()