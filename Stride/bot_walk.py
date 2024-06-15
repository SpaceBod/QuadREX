import time
import numpy as np
import pybullet as p
import pybullet_data
import math

from src.kinematic_model import RobotKinematics
from src.pybullet_debugger import PyBulletDebug
from src.gaitPlanner import TrotGait

def rendering(enable):
    """Enable/disable rendering"""
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, enable)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, enable)

def robot_init(dt, body_pos, fixed=False):
    """
    Initialize the robot in the PyBullet simulation environment.

    Parameters:
    dt (float): Time step for the simulation.
    body_pos (list): Initial position of the robot body [x, y, z].
    fixed (bool): Whether the robot body is fixed in place.

    Returns:
    tuple: Body ID and list of joint IDs.
    """
    physics_client = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    rendering(False)

    p.setGravity(0, 0, -2)
    p.setRealTimeSimulation(0)
    p.setPhysicsEngineParameter(
        fixedTimeStep=dt,
        numSolverIterations=100,
        enableFileCaching=0,
        numSubSteps=1,
        solverResidualThreshold=1e-10,
        erp=1e-1,
        contactERP=0.0,
        frictionERP=0.0,
    )

    # Add floor
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # Add robot
    body_id = p.loadURDF("body.urdf", body_pos, useFixedBase=fixed, baseOrientation=[1, 1, 1, 1])
    joint_ids = [
        p.getJointInfo(body_id, j)[0] for j in range(p.getNumJoints(body_id))
    ]

    # Robot properties
    max_velocity = 3.703  # rad/s
    for j in joint_ids:
        p.changeDynamics(body_id, j, lateralFriction=1e-5, linearDamping=0, angularDamping=0)
        p.changeDynamics(body_id, j, maxJointVelocity=max_velocity)

    rendering(True)
    return body_id, joint_ids

def robot_stepsim(body_id, body_pos, body_orn, body_to_feet):
    """
    Simulate one step of the robot in the PyBullet environment.

    Parameters:
    body_id (int): ID of the robot body in the simulation.
    body_pos (np.array): Position of the robot body [x, y, z].
    body_orn (np.array): Orientation of the robot body [roll, pitch, yaw].
    body_to_feet (np.array): Current positions of the feet relative to the body frame.

    Returns:
    np.array: Updated positions of the feet relative to the body frame.
    """
    max_force = 5  # N/m

    # Kinematics Model
    kinematics = RobotKinematics()
    fr_angles, fl_angles, br_angles, bl_angles, body_to_feet, valid = kinematics.solve(body_orn, body_pos, body_to_feet)
    if valid:
        # Move movable joints
        print(f"FR: {fr_angles * 180/math.pi}, FL: {fl_angles * 180/math.pi}, BR: {br_angles * 180/math.pi}, BL: {bl_angles * 180/math.pi}")
        for i in range(3):
            p.setJointMotorControl2(body_id, i, p.POSITION_CONTROL, targetPosition=fr_angles[i], force=max_force)
            p.setJointMotorControl2(body_id, 4 + i, p.POSITION_CONTROL, targetPosition=fl_angles[i], force=max_force)
            p.setJointMotorControl2(body_id, 8 + i, p.POSITION_CONTROL, targetPosition=br_angles[i], force=max_force)
            p.setJointMotorControl2(body_id, 12 + i, p.POSITION_CONTROL, targetPosition=bl_angles[i], force=max_force)

    p.stepSimulation()

    return body_to_feet

def robot_quit():
    """Disconnect from the PyBullet simulation."""
    p.disconnect()

if __name__ == "__main__":
    dt = 0.005
    body_id, joint_ids = robot_init(dt=dt, body_pos=[0, 0, 0.18], fixed=False)
    pybullet_debug = PyBulletDebug()
    trot = TrotGait()

    # Initial foot position
    x_dist, y_dist, height = 0.18, 0.15, 0.10
    body_to_feet_init = np.array([
        [x_dist / 2.0, -y_dist / 2.0, height],
        [x_dist / 2.0, y_dist / 2.0, height],
        [-x_dist / 2.0, -y_dist / 2.0, height],
        [-x_dist / 2.0, y_dist / 2.0, height],
    ])

    offset = np.array([0.5, 0.0, 0.0, 0.5])  # Offset between each foot step (FR, FL, BR, BL)
    T = 0.5  # Period of time (in seconds) for each step

    num_steps = 100000

    for _ in range(num_steps):
        pos, orn, leg_length, angle, leg_rotation, step_period, step_duration_asymmetry, offset = pybullet_debug.update_camera_and_get_robot_states(body_id)  # User input
        body_to_feet = trot.loop(leg_length, angle, leg_rotation, step_period, offset, body_to_feet_init, step_duration_asymmetry)  # Calculate feet coordinates for gait

        robot_stepsim(body_id, pos, orn, body_to_feet)  # Simulate robot to the target position

    robot_quit()
