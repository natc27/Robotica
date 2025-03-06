import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import yaml

class JoyButtonsPublisher(Node):
    def __init__(self):
        super().__init__('joy_buttons_publisher')

        # Cargar configuración
        self.declare_parameter('config_file', 'dualshock4_config.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Joy, 'joy_buttons', 10)

    def joy_callback(self, msg):
        filtered_msg = Joy()
        filtered_msg.header = msg.header  # Conservar el timestamp
        filtered_msg.axes = []  # No nos interesan los ejes
        filtered_msg.buttons = [0] * len(self.config['buttons'])

        for button_name, button_index in self.config['buttons'].items():
            if button_index < len(msg.buttons):
                filtered_msg.buttons[self.config['buttons'][button_name]] = msg.buttons[button_index]

        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyButtonsPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

