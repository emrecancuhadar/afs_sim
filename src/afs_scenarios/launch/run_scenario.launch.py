from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os, yaml
import datetime

def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}

def _resolve_paths(params: dict, base_dir: str, keys):
    out = dict(params)
    for k in keys:
        v = out.get(k, '')
        if isinstance(v, str) and v and not os.path.isabs(v):
            out[k] = os.path.join(base_dir, v)
    return out

def _make_nodes(context):
    scenario_name = context.launch_configurations.get('scenario', 'simple_hill.yaml')

    pkg_scen  = get_package_share_directory('afs_scenarios')
    pkg_world = get_package_share_directory('afs_world')
    pkg_env   = get_package_share_directory('afs_env')

    scen_path = os.path.join(pkg_scen, 'scenarios', scenario_name)
    cfg = _load_yaml(scen_path)

    common = cfg.get('common', {})

    # WORLD (map)
    wparams = cfg.get('afs_world', {})
    wparams.setdefault('resolution', common.get('resolution', 0.25))
    wparams = _resolve_paths(wparams, os.path.join(pkg_world, 'config'), ['map_png'])
    node_world = Node(
        package='afs_world', executable='map_pub', name='world_map_pub',
        output='screen', parameters=[wparams]
    )

    # ENV (rasters → grids)
    eparams = cfg.get('afs_env', {})
    eparams.setdefault('rows', common.get('rows', 40))
    eparams.setdefault('cols', common.get('cols', 40))
    eparams = _resolve_paths(
        eparams, os.path.join(pkg_env, 'config'),
        ['summer_red','summer_nir','winter_red','winter_nir','dem_tif','intensity_image']
    )
    node_env = Node(
        package='afs_env', executable='env_from_rasters', name='env_from_rasters',
        output='screen', parameters=[eparams]
    )

    # FIRE (cellular automaton)
    fparams = cfg.get('afs_fire', {})
    fparams.setdefault('rows', common.get('rows', 40))
    fparams.setdefault('cols', common.get('cols', 40))
    node_fire = Node(
        package='afs_fire', executable='fire_ca_node', name='fire_ca',
        output='screen', parameters=[fparams]
    )
    bag_dir = os.path.expanduser(f"~/afs_bags/fire_run_{datetime.datetime.now():%F_%H-%M-%S}")
    record = ExecuteProcess(
        cmd=[
            'ros2','bag','record',
            '-o', bag_dir,
            '-b','268435456',  # optional split (~256 MiB); keep or remove
            '--topics','/grid/fire_count','/grid/fire_state'
        ],
        output='screen'
    )
    return [node_world, node_env, node_fire]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='simple_hill.yaml',
            description='Scenario YAML in afs_scenarios/scenarios'
        ),
        OpaqueFunction(function=_make_nodes),
    ])
