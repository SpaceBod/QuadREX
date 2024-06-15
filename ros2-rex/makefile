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
lidar-run:
	@echo -e "${MAGENTA}Running LD19-LIDAR...${NC}"
	@ros2 launch ldlidar_node ldlidar.launch.py

# Activate Lidar Node
lidar-on:
	@echo -e "${MAGENTA}Activating LD19-LIDAR...${NC}"
	@ros2 lifecycle set /ldlidar_node configure
	@ros2 lifecycle set /ldlidar_node activate
	@ros2 launch cartographer_ros lidar_launch.py
# Clean target
clean:
	@echo -e "${MAGENTA}Cleaning up...${NC}"
	# Add your clean-up commands here
rex:
	@echo -e "${MAGENTA}Buliding package for REX...${NC}"
	@colcon build --packages-select rex
