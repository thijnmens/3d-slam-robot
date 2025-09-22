#!/usr/bin/env bash

# Update APT sources
sudo apt update

# Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# Build project
colcon build

# Source install
source install/setup.bash
