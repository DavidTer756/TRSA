import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

# --- Configuration ---
PACKAGE_NAME = 'trsa_lab1'
YAML_FILENAME = 'ost.yaml'
CONFIG_SUBDIR = 'calibration'
# ---------------------

class ImageRectifier(Node):
    def __init__(self):
        super().__init__('image_rectifier_node')
        
        # 1. Load Calibration Parameters from YAML
        self.load_calibration_params()

        # 2. Setup CvBridge for conversions
        self.br = CvBridge()
        
        # 3. Create Subscriber for raw/distorted image (Input)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # 4. Create Publisher for rectified image (Output)
        self.publisher = self.create_publisher(Image, '/camera/image_rect', 10)
        self.get_logger().info(f"Rectifier node initialized. Subscribing to /camera/image_raw and publishing to /camera/image_rect.")


    def load_calibration_params(self):
        # Construct the full path to the YAML file
        share_dir = get_package_share_directory(PACKAGE_NAME)
        yaml_path = os.path.join(share_dir, CONFIG_SUBDIR, YAML_FILENAME)

        self.get_logger().info(f"Loading calibration from: {yaml_path}")
        
        # Open and load the YAML file
        try:
            with open(yaml_path, 'r') as file:
                calib_data = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Calibration file not found at: {yaml_path}")
            # Use self.get_logger().fatal() or raise an exception to stop if file is critical
            return

        # Extract matrices and distortion coefficients
        # NumPy reshape is crucial for OpenCV to interpret the flat list correctly
        K_data = calib_data['camera_matrix']['data']
        D_data = calib_data['distortion_coefficients']['data']
        R_data = calib_data['rectification_matrix']['data']
        P_data = calib_data['projection_matrix']['data']
        
        self.K = np.array(K_data).reshape((3, 3))
        self.D = np.array(D_data)
        self.R = np.array(R_data).reshape((3, 3))
        self.P = np.array(P_data).reshape((3, 4))
        self.image_size = (calib_data['image_width'], calib_data['image_height'])

        # Pre-calculate the mapping needed for cv2.remap (efficient way to undistort)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, self.R, self.P, self.image_size, cv2.CV_32FC1
        )
        self.get_logger().info("Calibration parameters loaded successfully.")


    def image_callback(self, msg):
        # 1. Convert ROS Image message to OpenCV Mat
        try:
            # We assume bgr8 encoding based on your previous messages
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion failed: {e}")
            return
            
        # 2. Apply Rectification (Undistortion + Projection)
        # cv2.remap applies the pre-calculated transformation maps to the image
        rectified_image = cv2.remap(cv_image, self.map1, self.map2, cv2.INTER_LINEAR)

        # 3. Convert OpenCV Mat back to ROS Image message
        try:
            rectified_msg = self.br.cv2_to_imgmsg(rectified_image, encoding='bgr8')
            rectified_msg.header = msg.header # Keep the original timestamp and frame_id
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion failed (to ROS msg): {e}")
            return

        # 4. Publish the rectified image
        self.publisher.publish(rectified_msg)
        
        # Optional: Display the rectified image for visual verification
        cv2.imshow("Rectified Image", rectified_image)
        cv2.waitKey(33)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRectifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()