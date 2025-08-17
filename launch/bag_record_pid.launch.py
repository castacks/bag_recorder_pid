from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
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
        default_value=os.path.join(pkg_dir, 'config', 'tartan_rgbt.yaml'),
        description='Configuration file for bag record pid'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/logging',
        description='Logging directory'
    )
    
    mcap_qos_dir_arg = DeclareLaunchArgument(
        'mcap_qos_dir',
        default_value=os.path.join(pkg_dir, 'config'),
        description='MCAP QoS directory'
    )
    
    best_effort_qos_sub_arg = DeclareLaunchArgument(
        'best_effort_qos_sub',
        default_value=LaunchConfiguration('best_effort_qos_sub', default=True),
        description='Toggle Best-Effort QoS Setting of Subscriber'
    )


    # Create the node with launch configurations
    bag_record_node = Node(
        package='bag_record_pid',
        executable='bag_record_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'cfg_path': LaunchConfiguration('cfg_path'),
            'output_dir': LaunchConfiguration('output_dir'),
            'mcap_qos_dir': LaunchConfiguration('mcap_qos_dir'),
            'best_effort_qos_sub': LaunchConfiguration('best_effort_qos_sub')
        }],
        output='screen'
    )

    trigger_using_gpio = Node(
        package='bag_record_pid',
        executable='trigger_using_gpio',
        name='trigger_using_gpio',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'pin': 37,
            'pull': 'down',
            'publish_rate_hz': 1.0,
            'topic': 'bag_record_pid/set_recording_status',
            'active_low': False
        }]  
    )

    return LaunchDescription([
        # Add all argument declarations
        namespace_arg,
        node_name_arg,
        cfg_path_arg,
        output_dir_arg,
        mcap_qos_dir_arg,
        best_effort_qos_sub_arg,
        # Add the node
        bag_record_node,
        trigger_using_gpio
    ])

