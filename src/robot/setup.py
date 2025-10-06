import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'map'),
         glob('map/*.yaml')),
        (os.path.join('share', package_name, 'map'),
         glob('map/*.pgm')),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='group6',
    maintainer_email='group6@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "velocity_subscriber = robot.velocity_subscriber:main",
            "motor_controller = robot.motor_controller:main",
            "odometry_publisher = robot.odometry_publisher:main",
            "navigate_to_pose_client = robot.navigate_to_pose_client:main",
        ],
    },
)
