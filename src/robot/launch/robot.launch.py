import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


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

    # Split motor stack
    velocity_calculator = Node(
        package='robot',
        executable='velocity_calculator',
        name='velocity_calculator',
        output='screen'
    )
    motor_controller = Node(
        package='robot',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )
    odometry_node = Node(
        package='robot',
        executable='odometry_node',
        name='odometry_node',
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
            os.path.join(ros_dir, 'nav2_bringup', 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'False'}.items()
    )

    # Slam_toolbox with explicit params
    slam_toolbox_share = get_package_share_directory('robot')
    slam_params_file = os.path.join(slam_toolbox_share, 'config', 'slam_toolbox.yaml')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_dir, 'slam_toolbox', 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'slam_params_file': slam_params_file
        }.items()
    )

    # Visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(ros_dir, 'nav2_bringup', 'rviz', 'nav2_default_view.rviz')],
        output='screen'
    )

    return LaunchDescription([
        joy,
        joy2twist,
        velocity_calculator,
        motor_controller,
        odometry_node,
        rplidar,
        static_tf_base_to_laser,
        nav2,
        slam_toolbox,
        rviz2,
    ])
