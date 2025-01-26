from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
#this specific name should be used
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_spawner = Node(
        package="catch_them_all_py",
        executable="turtle_spawner"
    )

    turtle_controller= Node(
        package="catch_them_all_py",
        executable="turtle_controller"
    )
    
    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner)
    ld.add_action(turtle_controller)
    return ld