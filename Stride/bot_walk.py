import time
import numpy as np
import pybullet as p
import pybullet_data
import math

from src.kinematic_model import RobotKinematics
from src.pybullet_debugger import pybulletDebug
from src.gaitPlanner import TrotGait


def rendering(render):
    """Enable/disable rendering"""
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, render)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, render)


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
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    rendering(0)

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
    body_id = p.loadURDF(
        "body.urdf",
        body_pos,
        useFixedBase=fixed,
        baseOrientation=[1, 1, 1, 1],
    )
    joint_ids = []

    # Robot properties
    maxVel = 3.703  # rad/s
    for j in range(p.getNumJoints(body_id)):
        p.changeDynamics(body_id, j, lateralFriction=1e-5, linearDamping=0, angularDamping=0)
        p.changeDynamics(body_id, j, maxJointVelocity=maxVel)
        joint_ids.append(p.getJointInfo(body_id, j))

    rendering(1)
    return body_id, joint_ids


def robot_stepsim(body_id, body_pos, body_orn, body2feet):
    """
    Simulate one step of the robot in the PyBullet environment.

    Parameters:
    body_id (int): ID of the robot body in the simulation.
    body_pos (np.array): Position of the robot body [x, y, z].
    body_orn (np.array): Orientation of the robot body [roll, pitch, yaw].
    body2feet (np.array): Current positions of the feet relative to the body frame.

    Returns:
    np.array: Updated positions of the feet relative to the body frame.
    """
    maxForce = 5  # N/m

    # Kinematics Model
    kinematics = RobotKinematics()
    fr_angles, fl_angles, br_angles, bl_angles, body2feet_ = kinematics.solve(body_orn, body_pos, body2feet)

    # Move movable joints
    for i in range(3):
        p.setJointMotorControl2(body_id, i, p.POSITION_CONTROL, targetPosition=fr_angles[i], force=maxForce)
        p.setJointMotorControl2(body_id, 4 + i, p.POSITION_CONTROL, targetPosition=fl_angles[i], force=maxForce)
        p.setJointMotorControl2(body_id, 8 + i, p.POSITION_CONTROL, targetPosition=br_angles[i], force=maxForce)
        p.setJointMotorControl2(body_id, 12 + i, p.POSITION_CONTROL, targetPosition=bl_angles[i], force=maxForce)

    p.stepSimulation()

    return body2feet_


def robot_quit():
    """Disconnect from the PyBullet simulation."""
    p.disconnect()


if __name__ == "__main__":
    dT = 0.005
    bodyId, jointIds = robot_init(dt=dT, body_pos=[0, 0, 0.18], fixed=False)
    pybulletDebug = pybulletDebug()
    robotKinematics = RobotKinematics()
    trot = TrotGait()

    # Initial foot position
    Xdist, Ydist, height = 0.18, 0.15, 0.10
    bodytoFeet0 = np.array([
        [Xdist / 2.0, -Ydist / 2.0, height],
        [Xdist / 2.0, Ydist / 2.0, height],
        [-Xdist / 2.0, -Ydist / 2.0, height],
        [-Xdist / 2.0, Ydist / 2.0, height],
    ])

    offset = np.array([0.5, 0.0, 0.0, 0.5])  # Offset between each foot step (FR, FL, BR, BL)
    T = 0.5  # Period of time (in seconds) for each step

    N_steps = 100000

    for k_ in range(N_steps):
        pos, orn, L, angle, Lrot, T, sda, offset = pybulletDebug.cam_and_robotstates(bodyId)  # User input
        bodytoFeet = trot.loop(L, angle, Lrot, T, offset, bodytoFeet0, sda)  # Calculate feet coordinates for gait

        robot_stepsim(bodyId, pos, orn, bodytoFeet)  # Simulate robot to the target position

    robot_quit()
