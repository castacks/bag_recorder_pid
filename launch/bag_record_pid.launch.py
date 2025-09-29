
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('bag_record_pid')
    
    # Declare all launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the bag recorder node'
    )
    
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='bag_record_pid',
        description='Name for the bag recorder node'
    )
    
    cfg_path_arg = DeclareLaunchArgument(
        'cfg_path',
        default_value=os.path.join(pkg_dir, 'config', 'tartandriver.yaml'),
        description='Configuration file for bag record pid'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/logging',
        description='Logging directory'
    )
    
    storage_config_dir_arg = DeclareLaunchArgument(
        'storage_config_dir',
        default_value=os.path.join(pkg_dir, 'config'),
        description='MCAP QoS directory'
    )
    
    best_effort_qos_sub_arg = DeclareLaunchArgument(
        'best_effort_qos_sub',
        default_value=LaunchConfiguration('best_effort_qos_sub', default=True),
        description='Toggle Best-Effort QoS Setting of Subscriber. This is only used if in Heartbeat mode.'
    )

    heartbeat_mode_arg = DeclareLaunchArgument(
        'heartbeat_mode',
        default_value=LaunchConfiguration('heartbeat_mode', default=False),
        description='Know when to bag by listening to a heartbeat, like if interfaced with a GUI'
    )

    node_param_keys = [
        "cfg_path",
        "output_dir",
        "storage_config_dir",
        "best_effort_qos_sub",
        "heartbeat_mode",
    ]
    node_params = {k: LaunchConfiguration(k) for k in node_param_keys}

    # Create the node with launch configurations
    bag_record_node = Node(
        package='bag_record_pid',
        executable='bag_record_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        parameters=[node_params],
        output='screen',
    )

    return LaunchDescription([
        # Add all argument declarations
        namespace_arg,
        node_name_arg,
        cfg_path_arg,
        output_dir_arg,
        storage_config_dir_arg,
        best_effort_qos_sub_arg,
        heartbeat_mode_arg,
        bag_record_node
    ])

