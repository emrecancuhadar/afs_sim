from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('afs_fire')
    params = os.path.join(pkg_share, 'config', 'fire_ca.params.yaml')

    return LaunchDescription([
        Node(
            package='afs_fire',
            executable='fire_ca_node',
            name='fire_ca',
            output='screen',
            parameters=[params],
        ),
    ])
