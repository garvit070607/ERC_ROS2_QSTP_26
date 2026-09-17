# Week 1 - Obstacle Avoider

## Overview

This package implements a ROS 2 obstacle avoidance robot using
TurtleBot3 Burger, LiDAR sensing, and velocity control.

## Topics

### Published
- `/cmd_vel` - `geometry_msgs/msg/Twist`

### Subscribed
- `/scan` - `sensor_msgs/msg/LaserScan`

## Service

- `/toggle_robot` - `std_srvs/srv/SetBool`

The robot starts in the OFF state.

- `true` → Robot ON
- `false` → Robot OFF

## Behaviour

When the robot is ON:

- If no obstacle is detected within 1 metre in front, the robot
  moves in a spiral pattern.
- If an obstacle is detected within 1 metre, the robot compares
  the left and right LiDAR distances.
- The robot turns toward the side with more open space.

When the robot is OFF, both linear and angular velocities are set to zero.

## Running

Start the simulation:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch turtlebot3_gz_sim bringup.launch.py world:=obstacle.sdf
