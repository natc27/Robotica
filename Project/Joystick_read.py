#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # Mensaje estándar de ROS2 para joysticks
import pygame

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        
        # Inicializa el publicador del tópico /joystick_data
        self.publisher_ = self.create_publisher(Joy, '/joystick_data', 10)

        # Inicializa pygame y el joystick
        pygame.init()
        pygame.joystick.init()

        # Verifica que haya al menos un joystick conectado
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No se encontró un joystick conectado por USB.")
            return
        
        self.joystick = pygame.joystick.Joystick(0)  # Usa el primer joystick disponible
        self.joystick.init()
        self.get_logger().info(f"Joystick detectado: {self.joystick.get_name()}")

        # Crea un temporizador para actualizar el estado del joystick
        self.timer = self.create_timer(0.1, self.update_joystick)

    def update_joystick(self):
        """Lee los valores del joystick y los publica en ROS2."""
        pygame.event.pump()  # Actualiza los eventos de pygame

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        joy_msg.buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

        self.publisher_.publish(joy_msg)
        self.get_logger().info(f"Joystick Data: {joy_msg.axes}, {joy_msg.buttons}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo de joystick detenido.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
