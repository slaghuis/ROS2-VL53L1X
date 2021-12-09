# ROS2-VL53L1X
A ROS2 Node to run on a Raspberry Pi.  Reads the VL53L1X via I2C and publishes a sensor_msgs/msg/Range topic.  Uses the library written for the VL53L1X I2C Distance.

Development environment specifics:
Raspberry Pi 3
Ubuntu 20.04.2 LTS

## Installation
Wire the sensor as per manufacturers specifications.  Enable the i2c port on the Raspberry Pi.  A good exmple can be found at [Sparkfun Raspberry Pi SPI and I2C Tutorial](https://learn.sparkfun.com/tutorials/saprberry-pi-spi-and-i2c-tutorial/all).  Ensure that the user that will run the ROS node is in the correct group.
```
sudo usermod -aG i2c ubuntu
```

Clone this repo into the src directory of your ROS2 workspace. See the [ros2 tutorial](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) on how to create a workspace.
```
git clone https://github.com/slaghuis/ROS2-VL53L1X.git
```
Back in the root of your ROS workspace, build and install the package.  
```
colcon build --packages-select vl53l1x
. install/setup.bash
```
Run the package
```
ros2 run vl53l1x vl53l1x_node
```
See the output in a seperate terminal
```
ros2 topic echo vl53l1x/range
```
