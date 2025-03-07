import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DroidcamListener(Node):

    def __init__(self):
        super().__init__('droidcam_listener')
        self.subscription = self.create_subscription(
            Image,
            'droidcam_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('DroidcamListener iniciado y suscrito a /droidcam_image')

    def listener_callback(self, msg):
        try:
            # Convertir el mensaje de ROS2 a imagen de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("DroidCam Video Feed", cv_image)
            cv2.waitKey(1)  # Necesario para actualizar la ventana
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DroidcamListener()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()  # Cerrar ventana al apagar el nodo
    rclpy.shutdown()

if __name__ == '__main__':
    main()


