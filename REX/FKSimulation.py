import pybullet as p
import pybullet_data
import time
import math

# Start the simulation in GUI mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot_id = p.loadURDF(
    "body.urdf", basePosition=[0, 0, 0.2], baseOrientation=[1, 1, 1, 1]
)
p.setDebugObjectColor(robot_id, -1, [1, 0, 0, 0.7])
p.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=45, cameraPitch=-20, cameraTargetPosition=[0, 0, 0]
)

p.setGravity(0, 0, -9.81)

joint_configs = [
    {
        "name": "FR-Shoulder",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "right",
    },
    {
        "name": "FR-Femur",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "left",
    },
    {
        "name": "FR-Tibia",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "right",
    },
    {
        "name": "FL-Shoulder",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "left",
    },
    {
        "name": "FL-Femur",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "right",
    },
    {
        "name": "FL-Tibia",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "right",
    },
    {
        "name": "RL-Shoulder",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "left",
    },
    {
        "name": "RL-Femur",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "right",
    },
    {
        "name": "RL-Tibia",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "right",
    },
    {
        "name": "RR-Shoulder",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "right",
    },
    {
        "name": "RR-Femur",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "left",
    },
    {
        "name": "RR-Tibia",
        "min": 0,
        "max": 180,
        "offset": 90,
        "start": 0,
        "orientation": "right",
    },
]

sliders = []
for config in joint_configs:
    initial_value = math.degrees(config["start"]) + config["offset"]
    slider_id = p.addUserDebugParameter(
        config["name"], config["min"], config["max"], initial_value
    )
    sliders.append(slider_id)

# Run the simulation with interactive joint control
for _ in range(10000):
    for j, slider in enumerate(sliders):
        param_degrees = p.readUserDebugParameter(slider)
        if joint_configs[j]["orientation"] == "right":
            param_degrees = 180 - param_degrees  # Mirroring the angle for left joints
        param_radians = math.radians(param_degrees - joint_configs[j]["offset"])
        p.setJointMotorControl2(
            robot_id, j, p.POSITION_CONTROL, targetPosition=param_radians
        )
    p.stepSimulation()
    time.sleep(1 / 240)

# Disconnect the simulation (closes the window)
p.disconnect()
