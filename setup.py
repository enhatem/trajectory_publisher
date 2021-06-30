import os
from glob import glob
from setuptools import setup

package_name = 'trajectory_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elie',
    maintainer_email='e.na.hatem@gmail.com',
    description='ROS2 node that reads a trajectory from a .csv file publishes it using a message of type nav_msgs/Odometery',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher_node = trajectory_publisher.publisher_member_function:main',
        ],
    },
)
