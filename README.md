# REX: Autonomous Quadruped for Disaster Response

![QuadREX](https://github.com/SpaceBod/QuadREX/assets/26677677/8f891fbd-7d47-4390-b0d1-029d8dd80421)

## Overview

REX is a cost-effective, autonomous quadruped robot aimed at enhancing disaster response operations. Designed to navigate challenging terrains, it assists in search and rescue missions, emergency aid delivery, and survivor detection. The project integrates AI capabilities using IBM's Watson Assistant and features a modular design for ease of repair and maintenance.

## Features

- **Autonomous Navigation:** Navigates through terrains using SLAM and wavefront exploration.
- **Watson AI Integration:** Utilises IBM Watson Assistant for interaction and data collection from potential casualties.
- **Modular Design:** Allows for quick onsite repairs through field-replaceable units (FRUs).
- **Cost-Effective:** Designed with affordability in mind to make advanced robotics accessible to resource-constrained settings.
- **Simulation and Testing:** Python simulations using PyBullet for kinematics and gait cycle generation.
- **Depth Camera and MobilenetV2:** Uses depth camera and MobilenetV2 to locate people.
- **Web Dashboard:** A comprehensive dashboard for monitoring the robot and its interactions.


## Repository Contents

- **Simulations:** Python code for simulating the quadruped's kinematics and walking cycles.
- **Watson Assistant Code:** Scripts for integrating Watson AI capabilities.
- **Web Dashboard:** Code for the web-based dashboard used to monitor and control REX.

## Getting Started

### Prerequisites

- Python 3.x
- ROS2 Humble (Robot Operating System 2)
- PyBullet
- IBM Watson API credentials
- NodeJS

### Installation

1. **Clone the repository:**
    ```sh
    git clone https://github.com/SpaceBod/QuadREX.git
    cd QuadREX
    ```

2. **Set up IBM Watson Assistant:**
    - Obtain API credentials from IBM Watson.
    - Configure the credentials in the `watson_config.py` file.

3. **Set up the Web Dashboard:**
    - Navigate to the `web-app` directory.
    - Install dependencies: `npm install`.

4. **Install ROS2 and Dependencies:**
    Follow the ROS2 Humble installation guide from the [ROS2 documentation](https://docs.ros.org/en/humble/Installation.html).

5. **Install Nav2:**
    ```sh
    sudo apt update
    sudo apt install ros-humble-nav2-bringup
    ```

6. **Install Cartographer:**
    ```sh
    sudo apt update
    sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
    ```

7. **Install DepthAI:**
    ```sh
    # Install necessary ROS2 packages
    sudo apt update
    sudo apt install ros-humble-depthai ros-humble-depthai-examples
    
    # Set up the OAK-D Lite camera
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone --recurse-submodules https://github.com/luxonis/depthai-ros.git
    cd ..
    colcon build --symlink-install
    source install/setup.bash
    
    # Launch DepthAI example
    ros2 launch depthai_examples stereo_inertial_node.launch.py
    ```


### Running Simulations

Navigate to the `simulations` directory and run the simulation scripts:

```sh
cd simulations
python run_simulation.py
```
### Running the Dashboard

Navigate to the `web-app` directory and run web app:

```sh
cd web-app
npm start
```

### Running Path Planning and Wavefront Simulations
Navigate to the `navigation_sim` directory and run the path planning and wavefront simulation scripts:

```sh
cd navigation_sim
python path_planning_sim.py
python wavefront_sim.py

