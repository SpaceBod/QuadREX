import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Assuming these are defined or properly imported from your modules
from inverseKinematics import robotKinematics
from gait import walkGait  # or whatever gait you are using


def initialize_robot():
    """Initializes the robot in the PyBullet environment."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        "body.urdf",
        basePosition=[0, 0, 0.2],
        baseOrientation=[1, 1, 1, 1],
    )
    p.setGravity(0, 0, 0)
    return robot_id


def simulate_step(robot_id, angles):
    """Apply joint angles to the robot."""
    num_legs = 4
    joints_per_leg = 3  # Adjust based on your robot's design
    for leg in range(num_legs):
        for joint in range(joints_per_leg):
            joint_index = leg * joints_per_leg + joint
            p.setJointMotorControl2(
                robot_id,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=angles[leg, joint],
            )


robot_id = initialize_robot()
kinematics = robotKinematics()
gait_planner = walkGait()
initial_foot_positions = np.array(
    [
        [0.1, -0.1, 0.2],
        [0.1, 0.1, 0.2],
        [-0.1, -0.1, 0.2],
        [-0.1, 0.1, 0.2],
    ]
)
step_offset = np.array([0.5, 0, 0, 0.5])

# Main simulation loop
num_steps = 10000
for step in range(num_steps):
    # Compute new foot positions based on the gait
    foot_positions = gait_planner.loop(
        0.1, 0, 0, 1, step_offset, initial_foot_positions
    )
    # Get body position and orientation from the simulation state
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    # Calculate the necessary joint angles for the new positions
    angles, _ = kinematics.solve(orn, pos, foot_positions)

    # Apply the joint angles to the robot
    simulate_step(robot_id, angles)

    p.stepSimulation()
    time.sleep(1 / 240)

p.disconnect()
