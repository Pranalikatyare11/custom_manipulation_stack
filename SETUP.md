# Setup Guide

## 1. Clone Repository

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
git clone https://github.com/Pranalikatyare11/custom_manipulation_stack.git
cd custom_manipulation_stack
```

## 2. Automatic Setup

Run the setup script to install dependencies and build:

```bash
./setup.sh
source install/setup.bash
```

## 3. Manual Setup & Dependency Installation

If you prefer to install dependencies manually:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --ignore-src --from-paths src -y
```

Core dependencies:
- ROS 2 Humble
- MoveIt 2 & MoveIt Task Constructor (`ros-humble-moveit-task-constructor-core`)
- `ros2_control` and `ros2_controllers`
- Gazebo Fortress via `ros_gz` and `gz_ros2_control`
- `xacro`, `cv_bridge`, and `liburdfdom-tools`

Manual apt installation if needed:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-task-constructor-core \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-xacro \
  ros-humble-cv-bridge \
  liburdfdom-tools
```

## 4. Build Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Verification Commands

```bash
ros2 control list_controllers
ros2 action list | grep gripper_cmd
xacro src/custom_robot_description/urdf/custom_robot.urdf.xacro sim_gazebo:=true | check_urdf -
```
