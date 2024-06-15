from movement.quadruped import QuadrupedRobot

if __name__ == "__main__":
    robot = QuadrupedRobot()

    # Example of setting parameters
    robot.set_body_position([0, 0, 0])
    robot.set_body_orientation([0, 0, 0])
    robot.set_gait_parameters(
        leg_length=0.1,
        angle=0,
        leg_rotation=0,
        step_period=1.0,
        step_duration_asymmetry=0,
        offset=[0.5, 0.0, 0.0, 0.5]
    )

    # Run the robot
    robot.run(num_steps=100000)