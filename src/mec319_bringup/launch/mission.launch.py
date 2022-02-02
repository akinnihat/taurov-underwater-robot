from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    object_status_publisher = Node(
        package="mec319_pkg_cpp",
        executable="object_status_publisher"
    )

    movement_node = Node(
        package="mec319_pkg_py",
        executable="movement_node"
    )

    sonar_data_publisher = Node(
        package="mec319_pkg_py",
        executable="sonar_data_publisher"
    )

    ld.add_action(object_status_publisher)
    ld.add_action(movement_node)
    ld.add_action(sonar_data_publisher)

    return ld