from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='afs_clock',
             executable='sim_clock',
             name='sim_clock',
             output='screen',
             parameters=[{'dt': 0.05, 'start_time': 0.0, 'paused': False}]),
    ])
