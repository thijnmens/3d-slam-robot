from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Generates the launch description for the robot
    """

    # ROS Directory
    ros_dir = '/opt/ros/jazzy/share/'

    # Joystick Reader
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            # 'dev': '/dev/input/js0',
        }]
    )

    # Joystick to Twist converter
    joy2twist_share = get_package_share_directory('ros2_joy_twist')
    mappings = os.path.join(joy2twist_share, 'mappings.yaml')
    joy2twist = Node(
        package='ros2_joy_twist',
        executable='joy_to_twist',
        name='joy_to_twist',
        output='screen',
        parameters=[mappings],
    )

    # Motor driver
    motor_driver = Node(
        package='robot',
        executable='motor_driver',
        name='motor_driver',
        output='screen'
    )

    # Lidar node
    rplidar = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True
        }],
    )

    # Visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(ros_dir, 'rplidar_ros', 'rviz', 'rplidar.rviz')],
        output='screen'
    )

    return LaunchDescription([
        joy,
        joy2twist,
        motor_driver,
        rplidar,
        rviz2,
    ])