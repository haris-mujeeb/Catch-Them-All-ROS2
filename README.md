# Catch Them All

https://github.com/user-attachments/assets/4ce4d02a-6089-402e-a6e5-3274d896adc1


## Overview

The `catch_them_all` a small `turtlesim` for beginners to learn ROS2 basic functionalities. The Catcher node controls a turtle to move towards dynamically spawned target turtles and "catch" them by deleting their respective instances.


## Prerequisites

Before you begin, ensure you have met the following requirements:

- **ROS 2**: This package is compatible with Humble.
- **Colcon**: Make sure you have `colcon` installed for building the packages.

## Installation

Follow these steps to install the `catch_them_all` and `catch_them_all_bringup` packages:

1. **Clone this repository:**
   ```bash
   git clone https://github.com/haris-mujeeb/Catch-Them-All-ROS2.git
   ```

2. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. **Source the workspace:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

1. **Launch using the provided launch file:**
   ```bash
   ros2 launch catch_them_all_bringup catcher_app.launch.py
   ```

   This launch file will:
   - Start the Turtlesim simulation.
   - Spawn a catcher turtle and additional random turtles.
   - Launch the Catcher node to control the turtle.
   Note: To execute python code, use turtlesim_catch_them_all_py.launch.py instead of catcher_app.launch.py

2. **Manual commands:**
   Alternatively, you can run the nodes manually:
   ```bash
   ros2 run turtlesim turtlesim_node
   ros2 run catcher_them_all spawner
   ros2 run catcher_them_all catcher
   ```
   Note: Use turtle_spawner and turtle_controller for python code

## Configuration

The Catcher node can be customized with the following:
- **Default Turtle Name:** The node tracks and controls a specific turtle (`turtle1` by default).
- **Timers:** Control timers for movement and spawning can be adjusted in the code.

## Node Interface

### Published Topics
- `/turtle1/cmd_vel` (geometry_msgs/msg/Twist): Publishes velocity commands to control the turtle.
- `/alive_turtles` (my_robot_interfaces/msg/TurtleArray): Publishes an array with alive turtles with their object attributes (namely, x, y, theta).

### Subscribed Topics
- `/turtle1/pose` (turtlesim/msg/Pose): Subscribes to the turtle's position.

### Services
- `spawn_random_turtle` (turtlesim/srv/Spawn): Spawns turtles at random positions.
- `kill` (turtlesim/srv/Kill): Kills a target turtle after it is caught.

## Contributing

Contributions are welcome! Feel free to open issues or submit pull requests.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

