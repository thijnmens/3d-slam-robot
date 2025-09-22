#!/usr/bin/env bash

# Update APT sources
sudo apt update

# Install xterm
sudo apt install xterm

# Set environment variables
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

# Install dependencies
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# Build project
colcon build

# Source install
source install/setup.bash
