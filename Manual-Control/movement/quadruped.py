import numpy as np
from adafruit_servokit import ServoKit
from .kinematic_model import RobotKinematics
from .gaitPlanner import TrotGait
import time
import logging

class QuadrupedRobot:
    def __init__(self, dt=0.005, body_pos=[0, 0, 0], fixed=False):
        self.dt = dt
        self.body_pos = np.array(body_pos)
        self.fixed = fixed
        
        self.kinematics = RobotKinematics()
        self.trot_gait = TrotGait()
        
        # Initialize the ServoKit instance
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
            14: {"servo_id": 0, "min_angle": 45, "max_angle": 135, "offset": -8},   # BL Tibia
            12: {"servo_id": 1, "min_angle": 45, "max_angle": 135, "offset": 1},    # BL Coxa
            13: {"servo_id": 2, "min_angle": 45, "max_angle": 135, "offset": 3},    # BL Femur
            4:  {"servo_id": 3, "min_angle": 45, "max_angle": 135, "offset": 1},    # FL Coxa
            5:  {"servo_id": 4, "min_angle": 45, "max_angle": 135, "offset": 0},    # FL Femur
            6:  {"servo_id": 5, "min_angle": 45, "max_angle": 135, "offset": 0},    # FL Tibia
            1:  {"servo_id": 6, "min_angle": 45, "max_angle": 135, "offset": 4},    # FR Femur
            2:  {"servo_id": 7, "min_angle": 45, "max_angle": 135, "offset": 6},    # FR Tibia
            0:  {"servo_id": 8, "min_angle": 45, "max_angle": 135, "offset": -4},   # FR Coxa
            9:  {"servo_id": 9, "min_angle": 45, "max_angle": 135, "offset": -1},   # BR Femur
            8: {"servo_id": 10, "min_angle": 45, "max_angle": 135, "offset": -9},   # BR Coxa
            10: {"servo_id": 11, "min_angle": 45, "max_angle": 135, "offset": 15}   # BR Tibia
        }

        # Set up logging
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger(__name__)

    def set_body_position(self, pos):
        """Set the robot body position."""
        if len(pos) != 3:
            self.logger.error("Position must be a 3-element list or array")
            return
        self.body_pos = np.array(pos)

    def set_body_orientation(self, orn):
        """Set the robot body orientation (roll, pitch, yaw)."""
        if len(orn) != 3:
            self.logger.error("Orientation must be a 3-element list or array")
            return
        self.body_orn = np.array(orn)

    def set_gait_parameters(self, leg_length, angle, leg_rotation, step_period, step_duration_asymmetry, offset):
        """Set the gait parameters."""
        self.leg_length = leg_length
        self.angle = angle
        self.leg_rotation = leg_rotation
        self.step_period = step_period
        self.step_duration_asymmetry = step_duration_asymmetry
        self.offset = np.array(offset)

    def update_gait_parameters(self, params):
        """Update the gait parameters dynamically."""
        self.set_gait_parameters(
            params.get('leg_length', self.leg_length),
            params.get('angle', self.angle),
            params.get('leg_rotation', self.leg_rotation),
            params.get('step_period', self.step_period),
            params.get('step_duration_asymmetry', self.step_duration_asymmetry),
            params.get('offset', self.offset)
        )
    def update_body_orientation(self, orn):
        """Update the orientation parameters dynamically."""
        self.set_body_orientation(orn)
    def update_body_position(self, pos):
        """Update the position parameters dynamically."""
        self.set_body_position(pos)

    def calculate_feet_positions(self):
        """Calculate feet positions for the current gait parameters."""
        return self.trot_gait.loop(self.leg_length, self.angle, self.leg_rotation, self.step_period, self.offset, self.body_to_feet_init, self.step_duration_asymmetry)

    def apply_kinematics(self, body_to_feet):
        """
        Apply the kinematics model to get the joint angles for the feet positions.

        Returns:
        tuple: Joint angles for front-right, front-left, back-right, back-left legs.
        """
        try:
            fr_angles, fl_angles, br_angles, bl_angles, v0, v1, v2, v3, _ = self.kinematics.solve(self.body_orn, self.body_pos, body_to_feet)
            valid = v0 and v1 and v2 and v3
        except Exception as e:
            self.logger.error(f"Error in kinematics calculation: {e}")
            return None, None, None, None, False
        return fr_angles, fl_angles, br_angles, bl_angles, valid

    def step(self):
        """Perform a single step of the robot."""
        body_to_feet = self.calculate_feet_positions()
        fr_angles, fl_angles, br_angles, bl_angles, valid = self.apply_kinematics(body_to_feet)

        if not valid or any(joint is None for joint in [fr_angles, fl_angles, br_angles, bl_angles]):
            self.logger.error("Failed to calculate joint angles")
            return

        if valid:
            joint_angles = [fr_angles, fl_angles, br_angles, bl_angles]
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
        offset = servo_info['offset']

        # Ensure the angle is within the valid range
        if angle_degrees < min_angle:
            angle_degrees = min_angle
            self.logger.warning(f"Min angle reached for joint {joint_id}")
        elif angle_degrees > max_angle:
            angle_degrees = max_angle
            self.logger.warning(f"Max angle reached for joint {joint_id}")
        
        # Move the servo to the given angle
        try:
            self.kit.servo[servo_id].angle = angle_degrees + offset
        except Exception as e:
            self.logger.error(f"Failed to move servo {servo_id} to angle {angle_degrees}: {e}")

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
        
        # self.logger.info(f"FR: {fr_angles}, FL: {fl_angles}, BR: {br_angles}, BL: {bl_angles}")

    def run(self, num_steps=100000):
        """Run the robot for a given number of steps."""
        for _ in range(num_steps):
            self.step()
            # You could add a condition or mechanism here to check for new parameters
            # This could be through a shared memory space, a message queue, or another IPC mechanism
            time.sleep(self.dt)
