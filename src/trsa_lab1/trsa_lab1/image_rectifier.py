import rclpy
from rclpy.node import Node
import os
import yaml
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import ament_index_python.packages

class ImageRectifier(Node):
    def __init__(self):
        super().__init__('image_rectifier_node')
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        
        calibration_path = os.path.join( 
                ament_index_python.get_package_share_directory('trsa_lab1'), 'calibration','ost.yaml')
        
        self.load_camera_calibration(calibration_path)

        self.camera_model.fromCameraInfo(self.camera_info)
        
        self.publisher = self.create_publisher(Image, '/camera/image_rect', 10)
        
        self.raw_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,  
            10)
       
   
    def load_camera_calibration(self, calibration_path): 
        with open(calibration_path, 'r') as file: 
            self.camera_info = CameraInfo() 
            calibration_data = yaml.safe_load(file) 
            self.camera_info.width = calibration_data['image_width'] 
            self.camera_info.height = calibration_data['image_height'] 
            self.camera_info.distortion_model = calibration_data['distortion_model'] 
            self.camera_info.d = calibration_data['distortion_coefficients']['data'] 
            self.camera_info.k = calibration_data['camera_matrix']['data'] 
            self.camera_info.r = calibration_data['rectification_matrix']['data'] 
            self.camera_info.p = calibration_data['projection_matrix']['data'] 
            
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Make a copy of the input frame as the output buffer
            rectified_image = cv_image.copy()

            # Rectify the frame in-place
            self.camera_model.rectifyImage(cv_image, rectified_image)

            # Convert back to ROS Image msg
            rectified_msg = self.bridge.cv2_to_imgmsg(rectified_image, encoding='bgr8')

            # Publish rectified image
            self.publisher.publish(rectified_msg)

        except Exception as e:
            self.get_logger().error(f"Error rectifying image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageRectifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()