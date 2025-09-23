import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to your URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('robot'),
        'config',
        'RoombaBot.urdf'
    )

    # Path to your controller config file
    controller_config_path = os.path.join(
        get_package_share_directory('robot'),
        'config',
        'controller.yaml'
    )

    # Node to publish robot state from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_path).read()}]
    )

    # Node for the controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': open(urdf_file_path).read()}, controller_config_path],
        output='screen'
    )

    # Node to spawn the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Node to spawn the mecanum drive controller
    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        mecanum_drive_controller_spawner
    ])