import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool
import yaml
import collections

class JoyFilteredPublisher(Node):
    def __init__(self):
        super().__init__('joy_filtered_publisher')

        # Cargar configuración desde el archivo YAML
        self.declare_parameter('config_file', 'dualshock4_config.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        # Crear suscripción y publicaciones
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.axes_publisher = self.create_publisher(Float32MultiArray, 'filtered_axes', 10)
        self.buttons_publisher = self.create_publisher(Int32MultiArray, 'filtered_buttons', 10)
        self.r1_publisher = self.create_publisher(Bool, 'r1_button', 10)

        # Parámetro del filtro
        self.window_size = 5  # Tamaño de la ventana para la media móvil
        self.axis_history = {i: collections.deque(maxlen=self.window_size) for i in range(6)}

    def moving_average(self, index, new_value):
        """Aplica un filtro de media móvil a los valores de los ejes."""
        self.axis_history[index].append(new_value)
        return sum(self.axis_history[index]) / len(self.axis_history[index])

    def joy_callback(self, msg):
        # Leer el estado de R1
        r1_pressed = msg.buttons[self.config['buttons']['enable_send']] == 1

        # Publicar R1 en un tópico separado
        r1_msg = Bool()
        r1_msg.data = r1_pressed
        self.r1_publisher.publish(r1_msg)

        # Publicar los 3 botones especiales siempre
        button_msg = Int32MultiArray()
        button_msg.data = [
            msg.buttons[self.config['buttons']['special_1']],
            msg.buttons[self.config['buttons']['special_2']],
            msg.buttons[self.config['buttons']['special_3']],
        ]
        self.buttons_publisher.publish(button_msg)

        # Si R1 está presionado, publica los ejes (sticks + gatillos L2 y R2) con filtro
        if r1_pressed:
            axes_msg = Float32MultiArray()
            axes_msg.data = [
                self.moving_average(0, msg.axes[0]),  # Stick izquierdo horizontal
                self.moving_average(1, msg.axes[1]),  # Stick izquierdo vertical
                self.moving_average(2, msg.axes[3]),  # Stick derecho horizontal
                self.moving_average(3, msg.axes[4]),  # Stick derecho vertical
                self.moving_average(4, msg.axes[2]),  # Gatillo L2
                self.moving_average(5, msg.axes[5]),  # Gatillo R2
            ]
            self.axes_publisher.publish(axes_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyFilteredPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


