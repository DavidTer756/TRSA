import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageReader(Node):

    def __init__(self):
        super().__init__('image_reader_node')

        # Criar subscritor ao tópico /camera/image_raw
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.listener_callback,
            10)
        self.subscription  # evitar warning de variável não usada

        # Conversor ROS <-> OpenCV
        self.br = CvBridge()

    def listener_callback(self, msg):
        # Converter mensagem ROS2 -> OpenCV
        frame = self.br.imgmsg_to_cv2(msg, 'bgr8')

        # Mostrar imagem
        cv2.imshow("Image Reader", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
