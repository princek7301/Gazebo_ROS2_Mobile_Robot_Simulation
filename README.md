# ROS2 Gazebo Mobile Robot Simulation

This repository contains a simulation of a differential-drive mobile robot using ROS2 and Gazebo. The project implements autonomous navigation capabilities including SLAM (Simultaneous Localization and Mapping), LIDAR-based perception, and collision avoidance.

## Overview

This project demonstrates how to:
- Create a custom mobile robot model in Gazebo
- Implement differential drive control
- Set up LIDAR sensors for perception
- Perform real-time mapping and localization using SLAM
- Develop basic collision avoidance behaviors

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11
- Python 3.10+

## Installation

1. Install ROS2 Humble:
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

2. Install Gazebo and related packages:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

3. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

4. Clone this repository:
```bash
git clone https://github.com/yourusername/ros2_gazebo_mobile_robot.git
```

5. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Project Structure

```
ros2_gazebo_mobile_robot/
├── config/                  # Configuration files for navigation and SLAM
├── launch/                  # Launch files
├── meshes/                  # 3D models for the robot
├── models/                  # Gazebo model definitions
├── src/                     # ROS2 node source code
├── urdf/                    # Robot URDF description
├── worlds/                  # Gazebo world files
└── package.xml              # Package manifest
```

## Running the Simulation

### 1. Launch the robot in Gazebo:

```bash
ros2 launch ros2_gazebo_mobile_robot robot_gazebo.launch.py
```

### 2. Launch SLAM for mapping:

```bash
ros2 launch ros2_gazebo_mobile_robot slam.launch.py
```

### 3. Launch autonomous navigation:

```bash
ros2 launch ros2_gazebo_mobile_robot navigation.launch.py
```

### 4. Control the robot manually (optional):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Key Components

### Robot Model

The robot is a simple differential-drive platform with:
- Two powered wheels
- LIDAR sensor for obstacle detection
- Differential drive controller

### SLAM

The project uses slam_toolbox to create maps of the environment and localize the robot within them. The generated maps can be saved and reused for navigation.

### Navigation

The Navigation2 (Nav2) stack provides autonomous navigation capabilities including:
- Path planning
- Obstacle avoidance
- Goal setting

### Collision Avoidance

A simple collision avoidance node monitors LIDAR data and adjusts the robot's velocity to prevent collisions with obstacles.

## Customization

- Modify the robot model in the `urdf/` directory
- Create new environments in the `worlds/` directory
- Adjust navigation parameters in the `config/` directory

## Troubleshooting

- **Robot not moving**: Check that the differential drive controller is properly configured
- **SLAM not working**: Ensure the LIDAR sensor is publishing data on the correct topic
- **Navigation errors**: Check the parameters in config/nav2_params.yaml

## Next Steps

- Add a camera to the robot
- Implement object detection
- Create more complex navigation behaviors
- Add a graphical user interface for control

## License

This project is licensed under the MIT License - see the LICENSE file for details.
