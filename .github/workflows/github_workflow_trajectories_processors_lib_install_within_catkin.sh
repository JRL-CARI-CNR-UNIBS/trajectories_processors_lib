#!/bin/bash
set -e

# Source ROS Noetic setup
echo "Sourcing ROS Noetic"
source /opt/ros/noetic/setup.bash

# Define workspace paths
WORKSPACE_DIR=$(pwd)/openmore_ws
SRC_DIR=$WORKSPACE_DIR/src

# Create the workspace and source folder
echo "Setting up Catkin workspace at $WORKSPACE_DIR"
mkdir -p $SRC_DIR
cd $WORKSPACE_DIR

# Clone trajectories_processors_lib into the src folder 
cd $SRC_DIR

echo "Cloning trajectories_processors_lib repository into $SRC_DIR"
git clone https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib.git 

# Build the workspace
cd $WORKSPACE_DIR
echo "Building the Catkin workspace"
catkin config --extend /opt/ros/noetic
catkin config --install
catkin build

# Source the workspace setup
echo "Sourcing workspace setup"
source $WORKSPACE_DIR/devel/setup.bash
