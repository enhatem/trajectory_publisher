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
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='position_plot',
            arguments=['/odometry/pose/pose/position/x','/odometry/pose/pose/position/y','/odometry/pose/pose/position/z']
        ),
    ])