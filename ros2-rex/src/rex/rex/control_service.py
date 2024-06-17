import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
from rex.movement.quadruped import QuadrupedRobot

class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.orientation_subscription = self.create_subscription(
            Float32MultiArray,
            'rex/orientation',
            self.orientation_callback,
            10
        )
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.cmd_vel_subscription  # prevent unused variable warning
        self.orientation_subscription  # prevent unused variable warning
        self.robot = QuadrupedRobot()
        self.init_robot()
        self.velocity = 0
        self.angle = 0
        self.rotation = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.step_period = 0.4
        self.height = 0.0

    def init_robot(self):
        self.robot.set_body_position([0, 0, 0])
        self.robot.set_body_orientation([0, 0, 0])
        self.robot.set_gait_parameters(
            leg_length=0,
            angle=0,
            leg_rotation=0,
            step_period=0.4,
            step_duration_asymmetry=0,
            offset=[0.5, 0.0, 0.0, 0.5]
        )

    def cmd_vel_callback(self, msg):
        # Scale cmd_vel values to appropriate ranges for the robot
        self.angle, magnitude = self.calculate_angle_and_magnitude(msg.linear.x, msg.linear.y)

        if self.angle == 0:
            self.velocity = self.scale_value(magnitude, [0, 1], [0, 0.24])
            self.step_period = 0.35

        elif self.angle in [180, -180]:
            self.velocity = self.scale_value(magnitude, [0, 1], [0, 0.18])
            self.step_period = 0.25
        else:
            self.velocity = self.scale_value(magnitude, [0, 1], [0, 0.18])
            self.step_period = 0.25

        if abs(msg.angular.z) > 0.2:
            self.rotation = self.scale_value(msg.angular.z, [-1, 1], [-0.20, 0.20])
            self.velocity = self.scale_value(magnitude, [0, 1], [0, 0.24])
            self.step_period = 0.3
        else:
            self.rotation = 0.0

    def orientation_callback(self, msg):
        if len(msg.data) == 3:
            self.roll = self.scale_value(msg.data[0], [-1, 1], [-0.24, 0.24])
            self.pitch = self.scale_value(msg.data[1], [-1, 1], [-0.12, 0.12])
            self.yaw = self.scale_value(msg.data[2], [-1, 1], [-0.24, 0.24])

    def timer_callback(self):
        # Set robot parameters based on the received values
        self.robot.update_gait_parameters({
            'leg_length': self.velocity,
            'angle': self.angle,
            'leg_rotation': self.rotation,
            'step_period': self.step_period,
            'step_duration_asymmetry': 0,
            'offset': [0.5, 0.0, 0.0, 0.5]
        })

        self.robot.update_body_position([0, 0, self.height])
        self.robot.update_body_orientation([self.roll, self.pitch, self.yaw])

        self.robot.step()

    def calculate_angle_and_magnitude(self, x_axis, y_axis, tolerance=0.1):
        if abs(x_axis) < tolerance and abs(y_axis) < tolerance:
            return 0, 0
        angle = np.degrees(np.arctan2(-y_axis, x_axis))
        magnitude = np.sqrt(x_axis**2 + y_axis**2)
        
        # Snap angle to the nearest multiple of 45 degrees
        snapped_angle = round(angle / 45) * 45
        
        return snapped_angle, magnitude

    def scale_value(self, value, src_range, dst_range):
        src_min, src_max = src_range
        dst_min, dst_max = dst_range
        if value < src_min:
            value = src_min
        elif value > src_max:
            value = src_max
        scaled_value = dst_min + (float(value - src_min) / float(src_max - src_min) * (dst_max - dst_min))
        return 0 if abs(scaled_value) < 1e-6 else scaled_value

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriber()
    try:
        rclpy.spin(cmd_vel_subscriber)
    except KeyboardInterrupt:
        pass
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
