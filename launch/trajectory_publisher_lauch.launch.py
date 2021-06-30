from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_publisher',
            executable='trajectory_publisher_node',
            name='traj_pub',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
    ])