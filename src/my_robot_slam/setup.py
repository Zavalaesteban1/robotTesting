from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include scripts directory
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zlove',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for robot SLAM and navigation in maze environments',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_publisher = my_robot_slam.obstacle_publisher:main',
            'round1_real_controller = my_robot_slam.round1_realController:main',
        ],
    },
) 