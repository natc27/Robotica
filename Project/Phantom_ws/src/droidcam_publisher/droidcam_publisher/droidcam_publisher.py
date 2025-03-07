import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml

class DroidCamPublisher(Node):
    def __init__(self):
        super().__init__('droidcam_publisher')

        # Leer archivo de configuración directamente
        with open('src/droidcam_publisher/droidcam_params.yaml', 'r') as file:
            config = yaml.safe_load(file)

        self.droidcam_url = config['droidcam_url']
        self.publish_frequency = config['publish_frequency']

        self.publisher_ = self.create_publisher(Image, 'droidcam_image', 10)

        # Crear el timer con la frecuencia configurada
        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(self.droidcam_url)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DroidCamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


