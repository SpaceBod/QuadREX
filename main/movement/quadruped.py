import numpy as np
from adafruit_servokit import ServoKit

from .kinematic_model import RobotKinematics
from .gaitPlanner import TrotGait

class QuadrupedRobot:
    def __init__(self, dt=0.005, body_pos=[0, 0, 0], fixed=False):
        self.dt = dt
        self.body_pos = np.array(body_pos)
        self.fixed = fixed
        
        self.kinematics = RobotKinematics()
        self.trot_gait = TrotGait()
        
        # Initialise the ServoKit instance
        self.kit = ServoKit(channels=16)

        # Set the pulse width range for servos
        for i in range(12):
            self.kit.servo[i].set_pulse_width_range(500, 2500)

        # Initial foot position
        self.x_dist = 0.18
        self.y_dist = 0.15
        self.height = 0.10
        self.body_to_feet_init = np.array([
            [self.x_dist / 2.0, -self.y_dist / 2.0, self.height],
            [self.x_dist / 2.0, self.y_dist / 2.0, self.height],
            [-self.x_dist / 2.0, -self.y_dist / 2.0, self.height],
            [-self.x_dist / 2.0, self.y_dist / 2.0, self.height],
        ])
        
        self.offset = np.array([0.5, 0.0, 0.0, 0.5])  # Offset between each foot step (FR, FL, BR, BL)
        self.T = 0.5  # Period of time (in seconds) for each step

        # Dictionary mapping joints to servo ID numbers and their min/max angles
        self.servo_mapping = {
            14: {"servo_id": 0, "min_angle": 0, "max_angle": 180},   # BL Tibia
            12: {"servo_id": 1, "min_angle": 0, "max_angle": 180},   # BL Coxa
            13: {"servo_id": 2, "min_angle": 0, "max_angle": 180},   # BL Femur
            4:  {"servo_id": 3, "min_angle": 0, "max_angle": 180},   # FL Coxa
            5:  {"servo_id": 4, "min_angle": 0, "max_angle": 180},   # FL Femur
            6:  {"servo_id": 5, "min_angle": 0, "max_angle": 180},   # FL Tibia
            1:  {"servo_id": 6, "min_angle": 0, "max_angle": 180},   # FR Femur
            2:  {"servo_id": 7, "min_angle": 0, "max_angle": 180},   # FR Tibia
            0:  {"servo_id": 8, "min_angle": 0, "max_angle": 180},   # FR Coxa
            9:  {"servo_id": 9, "min_angle": 0, "max_angle": 180},   # BR Femur
            8: {"servo_id": 10, "min_angle": 0, "max_angle": 180},  # BR Coxa
            10: {"servo_id": 11, "min_angle": 0, "max_angle": 180}   # BR Tibia
        }

    def set_body_position(self, pos):
        """Set the robot body position."""
        self.body_pos = np.array(pos)

    def set_body_orientation(self, orn):
        """Set the robot body orientation (roll, pitch, yaw)."""
        self.body_orn = np.array(orn)

    def set_gait_parameters(self, leg_length, angle, leg_rotation, step_period, step_duration_asymmetry, offset):
        """Set the gait parameters."""
        self.leg_length = leg_length
        self.angle = angle
        self.leg_rotation = leg_rotation
        self.step_period = step_period
        self.step_duration_asymmetry = step_duration_asymmetry
        self.offset = np.array(offset)

    def calculate_feet_positions(self):
        """Calculate feet positions for the current gait parameters."""
        return self.trot_gait.loop(self.leg_length, self.angle, self.leg_rotation, self.step_period, self.offset, self.body_to_feet_init, self.step_duration_asymmetry)

    def apply_kinematics(self, body_to_feet):
        """
        Apply the kinematics model to get the joint angles for the feet positions.

        Returns:
        tuple: Joint angles for front-right, front-left, back-right, back-left legs.
        """
        fr_angles, fl_angles, br_angles, bl_angles, _ = self.kinematics.solve(self.body_orn, self.body_pos, body_to_feet)
        return fr_angles, fl_angles, br_angles, bl_angles

    def step(self):
        """Perform a single step of the robot."""
        body_to_feet = self.calculate_feet_positions()
        joint_angles = self.apply_kinematics(body_to_feet)
        self.send_joint_commands(joint_angles)
    
    def move_servo_to_angle(self, joint_id, angle_degrees):
        """
        Move the servo corresponding to the given joint ID to the specified angle in degrees,
        ensuring it stays within the min and max limits.

        Parameters:
        joint_id (int): ID of the joint.
        angle_degrees (float): Target angle in degrees.
        """
        servo_info = self.servo_mapping[joint_id]
        servo_id = servo_info['servo_id']
        min_angle = servo_info['min_angle']
        max_angle = servo_info['max_angle']

        # Ensure the angle is within the valid range
        if angle_degrees < min_angle:
            angle_degrees = min_angle
        elif angle_degrees > max_angle:
            angle_degrees = max_angle

        # Move the servo to the given angle
        self.kit.servo[servo_id].angle = angle_degrees

    def send_joint_commands(self, joint_angles):
        """
        Send joint commands to the robot's motors.

        Parameters:
        joint_angles (tuple): Joint angles for front-right, front-left, back-right, back-left legs.
        """
        fr_angles, fl_angles, br_angles, bl_angles = joint_angles
        
        # Front Right (FR) Servos
        self.move_servo_to_angle(0, fr_angles[0])  # FR Coxa
        self.move_servo_to_angle(1, fr_angles[1])  # FR Femur
        self.move_servo_to_angle(2, fr_angles[2])  # FR Tibia

        # Front Left (FL) Servos
        self.move_servo_to_angle(4, fl_angles[0])  # FL Coxa
        self.move_servo_to_angle(5, fl_angles[1])  # FL Femur
        self.move_servo_to_angle(6, fl_angles[2])  # FL Tibia

        # Back Right (BR) Servos
        self.move_servo_to_angle(8, br_angles[0])  # BR Coxa
        self.move_servo_to_angle(9, br_angles[1])  # BR Femur
        self.move_servo_to_angle(10, br_angles[2])  # BR Tibia

        # Back Left (BL) Servos
        self.move_servo_to_angle(12, bl_angles[0])  # BL Coxa
        self.move_servo_to_angle(13, bl_angles[1])  # BL Femur
        self.move_servo_to_angle(14, bl_angles[2])  # BL Tibia
        
        print(f"FR: {fr_angles}, FL: {fl_angles}, BR: {br_angles}, BL: {bl_angles}")

    def run(self, num_steps=100000):
        """Run the robot for a given number of steps."""
        for _ in range(num_steps):
            self.step()
