# ROS2-VL53L1X
A ROS2 Node to run on a Raspberry Pi.  Reads the VL53L1X via I2C and publishes a sensor_msgs/msg/Range.  Uses the library written for the VL53L1X I2C Distance sensor.

Development environment specifics:
Raspberry Pi 3
Ubuntu 20.04.2 LTS

Node can be improved by adding parameters for the i2c sensor address, distance mode and measurement timing budget.
