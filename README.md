# ballCollectorRobot[![Build Status](https://travis-ci.org/ytlei/ballCollectorRobot.svg?branch=master)](https://travis-ci.org/ytlei/ballCollectorRobot)

Ball Collector Robot project

## Overview
build a simulation of the ball collector robot on ROS. It will be built with simple differential drive(turtlebot like) robot in ROS. The ball collector robot can find the nearest ball and goes to the ball and push them back to the corner for users to collect easily.

The Robot: Use turtlebot as a foundation and use the sensor on it to see the environment and objects. After Acquiring the object, generate a path using navigation technique to collecting the ball.
The environment: create a simulated tennis court with balls placed randomly and with
walls on four sides.
Tools may include Gazebo Simulator, Turtlebot module, ROS, and RVIZ for visualization

## Author
Yi-ting Lei
UMD Masters student in Robotics

## Link to AIP Sheet
[Iterative Process](https://docs.google.com/a/umd.edu/spreadsheets/d/1lzo7GK30SF71DnFegieKbuoOMx1Z_jcZFoYa5Z1Lw1A/edit?usp=sharing)

## Dependencies
 * ROS Kinetic
 * navigation modules
 * gazebo turtlebot pakcages
 * geometry modules
 * google test for unit tests

## Building

```
# into catkin workspace src/ folder
git clone https://github.com/ytlei/ballCollectorRobot.git

# from catkin workspace root
catkin_make 
```
## Issues

## Demo

## Doxygen Documentation

To generate the doxygen documentation from the root directory:

```
doxygen doxygen.config
```

This will create the documentation in the docs directory.

## TODO

## License
MIT open-source license, see LICENSE.TXT
