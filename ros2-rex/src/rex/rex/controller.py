import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import pygame
import sys
import time

class JoystickController(Node):

    def __init__(self):
        super().__init__('joystick_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.orientation_publisher_ = self.create_publisher(Float32MultiArray, 'rex/orientation', 10)
        self.timer = self.create_timer(0.05, self.publish_data)
        self.init_joystick()
        self.velocity = Twist()

        self.mode = "move"

    def init_joystick(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().info("No joystick connected")
            sys.exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Joystick name: {self.joystick.get_name()}")

    def publish_data(self):
        pygame.event.pump()
        left_stick_x = self.joystick.get_axis(0)
        left_stick_y = -self.joystick.get_axis(1)
        right_stick_x = self.joystick.get_axis(2)
        right_stick_y = -self.joystick.get_axis(3)

        button_b = self.joystick.get_button(1)

        # Apply deadzone to joystick inputs
        if abs(left_stick_x) < 0.1:
            left_stick_x = 0
        if abs(left_stick_y) < 0.1:
            left_stick_y = 0
        if abs(right_stick_x) < 0.1:
            right_stick_x = 0
        if abs(right_stick_y) < 0.1:
            right_stick_y = 0

        if button_b:
                self.mode = "move" if self.mode == "idle" else "idle"
                print(f"Mode changed to: {self.mode}")
                time.sleep(0.3)

        if self.mode == "move":
            self.velocity.linear.x = left_stick_y * 1.0     # Forward/backward
            self.velocity.linear.y = left_stick_x * 1.0     # Strafe Left/right
            self.velocity.angular.z = right_stick_x * 1.0   # Rotate Left/right
            orientation = Float32MultiArray()
            orientation.data = [0.0, 0.0, 0.0]
        else:
            self.velocity.linear.x = 0.0
            self.velocity.linear.y = 0.0
            self.velocity.angular.z = 0.0
            orientation = Float32MultiArray()
            orientation.data = [float(-left_stick_x), float(-left_stick_y), float(-right_stick_x)]  # roll, pitch, yaw

        self.publisher_.publish(self.velocity)
        self.orientation_publisher_.publish(orientation)

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    try:
        rclpy.spin(joystick_controller)
    except KeyboardInterrupt:
        pass
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
