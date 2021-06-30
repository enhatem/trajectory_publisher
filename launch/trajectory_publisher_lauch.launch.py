from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='position_plot',
            arguments=['/desired_pose/pose/pose/position/x','/desired_pose/pose/pose/position/y','/desired_pose/pose/pose/position/z']
        ),

        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='orientation_plot',
            arguments=['/desired_pose/pose/pose/orientation/w','/desired_pose/pose/pose/orientation/x','/desired_pose/pose/pose/orientation/y','/desired_pose/pose/pose/orientation/z']
        ),

        Node(
            package='trajectory_publisher',
            executable='trajectory_publisher_node',
            name='traj_pub',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        
    ])