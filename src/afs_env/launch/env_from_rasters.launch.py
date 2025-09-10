from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('afs_env')
    cfg = os.path.join(pkg_share, 'config')
    return LaunchDescription([
        Node(
            package='afs_env',
            executable='env_from_rasters',
            name='env_from_rasters',
            output='screen',
            parameters=[{
                'rows': 40,
                'cols': 40,
                'summer_red':   os.path.join(cfg, 'summer_red.tiff'),
                'summer_nir':   os.path.join(cfg, 'summer_nir.tiff'),
                'winter_red':   os.path.join(cfg, 'winter_red.tiff'),
                'winter_nir':   os.path.join(cfg, 'winter_nir.tiff'),
                'dem_tif':      os.path.join(cfg, 'dem.tif'),
                'intensity_image': '',   # optional
            }]
        ),
    ])
