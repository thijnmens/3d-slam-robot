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
    package_dir = get_package_share_directory('robot')

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

    # Static transform base_link -> laser (adjust translation/rotation as needed)
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_laser',
        output='screen',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser']
    )

    # nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_dir, 'nav2_bringup', 'launch', 'bringup_launch.py')
        ),
        launch_arguments={'use_sim_time': 'False', 'use_localization': 'True', 'map': f'{package_dir}/map/map.yaml'}.items()
    )

    return LaunchDescription([
        motor_driver,
        rplidar,
        static_tf_base_to_laser,
        nav2,
    ])