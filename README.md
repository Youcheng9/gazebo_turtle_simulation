# Auto Obstacle Avoid Robot

A ROS 2 Humble workspace for a TurtleBot3 obstacle avoidance project in Gazebo.

The main controller is written in C++ and uses `LaserScan` data from `/scan` to publish velocity commands on `/cmd_vel`. The behavior is tuned to:

- move forward when the path is clear
- detect obstacles in front of the robot
- avoid scraping side walls by checking front-corner and side sectors
- choose the turn direction based on the clearer side
- keep a committed turn direction until the path is safely clear

## Workspace Structure

This project currently contains two ROS 2 packages:

- `obstacle_avoid_cpp`
  Contains the obstacle avoidance node implemented in C++.
- `obstacle_avoid_bringup`
  Placeholder bringup package for launch/setup organization.

## Requirements

- Ubuntu with ROS 2 Humble installed
- TurtleBot3 simulation packages
- Gazebo Classic with ROS 2 integration

Typical dependencies:

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `turtlebot3_gazebo`
- `gazebo_ros`

## Run in Simulation

Set the TurtleBot3 model first:

```bash
export TURTLEBOT3_MODEL=burger
```

Start a TurtleBot3 Gazebo world in one terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In a second terminal, source ROS 2 and the workspace, then run the controller:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run obstacle_avoid_cpp obstacle_avoid_node
```

## How the Controller Works

The node subscribes to:

- `/scan`

The node publishes:

- `/cmd_vel`

The controller splits the laser scan into several sectors:

- front
- front-left
- front-right
- left side
- right side
- left open space
- right open space

It then uses:

- minimum distance sectors for safety
- average distance sectors to choose the more open direction
- separate enter and exit thresholds to reduce oscillation
- committed turn memory so the robot does not keep switching left/right every cycle

