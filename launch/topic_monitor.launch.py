
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
        description='Namespace for the topic monitor'
    )
    
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='topic_monitor',
        description='Name for the topic monitor'
    )
    
    cfg_path_arg = DeclareLaunchArgument(
        'cfg_path',
        default_value=os.path.join(pkg_dir, 'config', 'topic_rates.yaml'),
        description='Configuration file for topic monitor'
    )
    
    
    node_param_keys = [
        "cfg_path"
    ]
    node_params = {k: LaunchConfiguration(k) for k in node_param_keys}

    # Create the node with launch configurations
    topic_monitor_node = Node(
        package='bag_record_pid',
        executable='topic_monitor',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        parameters=[node_params],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        # Add all argument declarations
        namespace_arg,
        node_name_arg,
        cfg_path_arg,
        topic_monitor_node
    ])

