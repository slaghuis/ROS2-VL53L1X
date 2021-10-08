# ROS2-VL53L1X
A ROS2 Node to run on a Raspberry Pi.  Reads the VL53L1X via I2C and publishes a sensor_msgs/msg/Range topic.  Uses the library written for the VL53L1X I2C Distance.

Development environment specifics:
Raspberry Pi 3
Ubuntu 20.04.2 LTS

Node can be improved by adding parameters for the i2c sensor address, distance mode and measurement timing budget.

## Installation
Clone this repo into the src directory of your ROS2 workspace. See the [ros2 tutorial](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) on how to create a workspace.
```
git clone https://github.com/slaghuis/ROS2-VL53L1X.git
```
Back in the root of your workspace, build and install the package.  
```
colcon build --packages-select vl53l1x
. install/setup.bash
```
Run the package
```
ros2 run vl53l1x vl53l1x_node
```
See the output
```
ros2 topic echo vl53l1x/range
```
