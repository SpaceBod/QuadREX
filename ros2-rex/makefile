# Define the shell
SHELL := /bin/bash

# Define variables
SETUP_SCRIPT = ~/.bashrc
GREEN = \033[0;32m
BLUE = \033[0;34m
MAGENTA = \033[0;35m
NC = \033[0m # No Color

# Default target
all: setup compile auth

# Target to source the setup script
setup:
	@echo -e "${MAGENTA}Sourcing setup script...${NC}"
	@source $(SETUP_SCRIPT)

# Authenticate the serial and I2C ports
auth:
	@echo -e "${MAGENTA}Applying chmod permissions...${NC}"
	@sudo chmod 666 /dev/serial0
	@sudo chmod 666 /dev/i2c-1

# Build the workspace
compile:
	@echo -e "${MAGENTA}Building Packages...${NC}"
	@colcon build

# Run Lidar Node
lidar:
	@echo -e "${MAGENTA}Running LD19-LIDAR...${NC}"
	@ros2 launch ldlidar_node ldlidar_with_mgr.launch.py

camera:
	@echo -e "${MAGENTA}Running OAK-D LITE...${NC}"
	@ros2 launch depthai_examples mobile_tracker.launch.py camera_model:=OAK-D-LITE monoResolution:=480p

tracker:
	@echo -e "${MAGENTA}Running Tracker...${NC}"
	@ros2 run rex tracklets_parser	

human:
	@echo -e "${MAGENTA}Running Human Detection...${NC}"
	ros2 run rex human_detect

control:
	@echo -e "${MAGENTA}Starting REX Control Service...${NC}"
	@ros2 run rex control_service

# Activate Navigation Node
nav:
	@echo -e "${MAGENTA}Launching Navigation...${NC}"
	@ros2 launch nav navigation.launch.py

# Activate Cartographer Node
carto:
	@echo -e "${MAGENTA}Launching Cartographer...${NC}"
	@ros2 launch nav cartographer.launch.py

# Run RViz2
rviz:
	@echo -e "${MAGENTA}Launch RViz...${NC}"
	@ros2 launch nav rviz.launch.py

rex:
	@echo -e "${MAGENTA}Buliding package for REX...${NC}"
	@colcon build --packages-select rex
