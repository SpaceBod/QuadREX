import pybullet as p
import time
import numpy as np
import sys

def round_to_zero(val, epsilon=1e-7):
    """Round small values to zero based on a threshold epsilon."""
    return 0. if abs(val) < epsilon else val

class PyBulletDebug:
    def __init__(self):
        # Camera parameters to be able to yaw, pitch, and zoom the camera (Focus remains on the robot)
        self.camera_yaw = 90
        self.camera_pitch = -7
        self.camera_distance = 0.30
        time.sleep(0.5)
        
        self.params = {
            "pos_x": p.addUserDebugParameter("x", -0.02, 0.02, 0.),
            "pos_y": p.addUserDebugParameter("y", -0.02, 0.02, 0.),
            "pos_z": p.addUserDebugParameter("z", -0.02, 0.02, 0.),
            "roll": p.addUserDebugParameter("roll", -np.pi/4, np.pi/4, 0.),
            "pitch": p.addUserDebugParameter("pitch", -np.pi/4, np.pi/4, 0.),
            "yaw": p.addUserDebugParameter("yaw", -np.pi/4, np.pi/4, 0.),
            "leg_length": p.addUserDebugParameter("L", -0.50, 1, 0.),
            "leg_rotation": p.addUserDebugParameter("Lrot", -1.50, 1.50, 0.),
            "walk_angle": p.addUserDebugParameter("angleWalk", -180., 180., 0.),
            "step_period": p.addUserDebugParameter("stepPeriod", 0.1, 3., 1.0),
            "step_duration_asymmetry": p.addUserDebugParameter("step_dur_asym", -2, 2., 0.0),
            "trot": p.addUserDebugParameter("TROT", 0, 1, 0),
            "bound": p.addUserDebugParameter("BOUND", 0, 1, 0),
        }
        
    def update_camera_and_get_robot_states(self, box_id):
        # Orientation of camera
        cube_position, _ = p.getBasePositionAndOrientation(box_id)
        p.resetDebugVisualizerCamera(cameraDistance=self.camera_distance, cameraYaw=self.camera_yaw, cameraPitch=self.camera_pitch, cameraTargetPosition=cube_position)
        keys = p.getKeyboardEvents()
        
        # Update camera based on keyboard input
        self.update_camera(keys)
        
        # Read position from debug parameters
        position = np.array([p.readUserDebugParameter(self.params[axis]) for axis in ["pos_x", "pos_y", "pos_z"]])
        orientation = np.array([p.readUserDebugParameter(self.params[axis]) for axis in ["roll", "pitch", "yaw"]])
        leg_length = round_to_zero(p.readUserDebugParameter(self.params["leg_length"]))
        leg_rotation = p.readUserDebugParameter(self.params["leg_rotation"])
        walk_angle = p.readUserDebugParameter(self.params["walk_angle"])
        step_period = p.readUserDebugParameter(self.params["step_period"])
        step_duration_asymmetry = p.readUserDebugParameter(self.params["step_duration_asymmetry"])
        trot = p.readUserDebugParameter(self.params["trot"])
        bound = p.readUserDebugParameter(self.params["bound"])
        
        offset = self.calculate_offset(trot, bound)
        
        return position, orientation, leg_length, walk_angle, leg_rotation, step_period, step_duration_asymmetry, offset

    def update_camera(self, keys):
        if keys.get(65296):  # RIGHT
            self.camera_yaw -= 1
        if keys.get(65295):  # LEFT
            self.camera_yaw += 1
        if keys.get(65297):  # UP    
            self.camera_pitch -= 1
        if keys.get(65298):  # DOWN
            self.camera_pitch += 1
        if keys.get(45):  # +
            self.camera_distance += 0.01
        if keys.get(61):  # -
            self.camera_distance -= 0.01
        if keys.get(27):  # ESC
            p.disconnect()
            time.sleep(2)
            sys.exit()
        
        for key, status in keys.items():
            if status & p.KEY_IS_DOWN:
                print(f"Key {key} pressed")
    
    @staticmethod
    def calculate_offset(trot, bound):
        if trot == 1:
            return [0.5, 0., 0., 0.5]
        if bound == 1:
            return [0.5, 0.5, 0., 0.]
        return [0.5, 0., 0., 0.5]
