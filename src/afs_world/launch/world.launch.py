from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='afs_world',
            executable='map_pub',
            name='world_map_pub',
            output='screen',
            parameters=[{
                'map_png': 'config/map_40x40.png',
                'resolution': 0.25,
                'origin_xy': [0.0, 0.0],
                'occupied_threshold': 0.5,
                'frame_id': 'map'
            }]
        ),
    ])
