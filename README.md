# Catch Them All

## Overview

The `catch_them_all` a small `turtlesim` for beginners to learn ROS2 basic functionalities. The Catcher node controls a turtle to move towards dynamically spawned target turtles and "catch" them by deleting their respective instances.


## Prerequisites

Before you begin, ensure you have met the following requirements:

- **ROS 2**: This package is compatible with Humble.
- **Colcon**: Make sure you have `colcon` installed for building the packages.

## Installation

Follow these steps to install the `catch_them_all` and `catch_them_all_bringup` packages:

1. **Create a ROS 2 Workspace**

   If you don't have a ROS 2 workspace set up, create one using the following commands:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

   ```

2. **Clone this repository:**
   ```bash
   git clone https://github.com/your_username/catcher_node.git
   ```

3. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. **Source the workspace:**
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

2. **Manual commands:**
   Alternatively, you can run the nodes manually:
   ```bash
   ros2 run turtlesim turtlesim_node
   ros2 run catcher_node catcher_node
   ```

## Configuration

The Catcher node can be customized with the following:
- **Default Turtle Name:** The node tracks and controls a specific turtle (`turtle1` by default).
- **Timers:** Control timers for movement and spawning can be adjusted in the code.

## Node Interface

### Published Topics
- `/turtle1/cmd_vel` (geometry_msgs/msg/Twist): Publishes velocity commands to control the turtle.

### Subscribed Topics
- `/turtle1/pose` (turtlesim/msg/Pose): Subscribes to the turtle's position.

### Services
- `spawn_random_turtle` (turtlesim/srv/Spawn): Spawns turtles at random positions.
- `kill` (turtlesim/srv/Kill): Kills a target turtle after it is caught.

## Contributing

Contributions are welcome! Feel free to open issues or submit pull requests.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

