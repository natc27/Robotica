import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import yaml

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')

        # Cargar configuración
        self.declare_parameter('config_file', 'dualshock4_config.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        # Suscripción a /joy
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publicador a /turtle1/cmd_vel
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def joy_callback(self, msg):
        twist = Twist()

        # Movimiento en X (adelante/atrás)
        twist.linear.x = msg.axes[self.config['axes']['linear_x']] * self.config['scales']['linear_x']

        # Movimiento lateral (izquierda/derecha)
        twist.linear.y = msg.axes[self.config['axes']['linear_y']] * self.config['scales']['linear_y']

        # Rotación Z
        twist.angular.z = msg.axes[self.config['axes']['angular_z']] * self.config['scales']['angular_z']

        # Turbo duplica velocidades
        if msg.buttons[self.config['buttons']['turbo']] == 1:
            twist.linear.x *= 2
            twist.linear.y *= 2
            twist.angular.z *= 2

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

