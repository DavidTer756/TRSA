import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge 
import numpy as np
import ament_index_python

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.0667, self.timer_callback)  # 15 FPS
        self.cap = cv2.VideoCapture("/home/davidter/ros2_ws/src/trsa_lab1/video/trsa_video.mov")  # 0 = webcam 
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Video stream ended")
            rclpy.timer.cancel()
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erro ao converter e publicar frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
