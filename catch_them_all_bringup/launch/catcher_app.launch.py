from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  ld = LaunchDescription()

  turtlesim_node = Node(
    package="turtlesim",
    executable="turtlesim_node"
  )


  catcher_node = Node(
    package="catch_them_all",
    executable="catcher"
  )

  spawner_node = Node(
    package="catch_them_all",
    executable="spawner"
  )
  ld.add_action(turtlesim_node)
  ld.add_action(spawner_node)
  ld.add_action(catcher_node)
  return ld