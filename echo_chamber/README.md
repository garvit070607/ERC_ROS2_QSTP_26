# Week 0 - Echo Chamber

## Objective

The objective of this assignment was to create a basic ROS 2 publisher and subscriber system using Python.

## Package

Package name:

`echo_chamber`

## Nodes

### Talker

The `talker` node:

- Publishes random floating-point numbers between 0.0 and 100.0.
- Publishes once every second.
- Uses the `std_msgs/msg/Float32` message type.
- Publishes on the `/random_number` topic.

### Listener

The `listener` node:

- Subscribes to the `/random_number` topic.
- Receives the published floating-point number.
- Multiplies the received value by 2.
- Logs both the received and multiplied values.

## ROS Communication

```text
/talker
    |
    | std_msgs/msg/Float32
    v
/random_number
    |
    v
/listener
