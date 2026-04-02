"""
Launch file for Teensy Wave Trigger Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.insert(0, get_package_share_directory("ros_rec") + "/launch")
from composable import make_recorder_nodes  # type: ignore
from launch_ros.actions import ComposableNodeContainer, Node


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/teensy_trigger',
        description='Serial port for Teensy communication (shared by trigger + PPS)'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Serial baud rate'
    )

    # Trigger GPIO (30 Hz wave, gpiochip1:8)
    gpio_chip_arg = DeclareLaunchArgument(
        'trigger_gpio_chip',
        default_value='/dev/gpiochip1',
        description='GPIO chip for 30 Hz trigger edges'
    )

    gpio_line_arg = DeclareLaunchArgument(
        'trigger_gpio_line',
        default_value='8',
        description='GPIO line for 30 Hz trigger edges'
    )

    frame_id_arg = DeclareLaunchArgument(
        'trigger_frame_id',
        default_value='teensy_trigger',
        description='Frame ID for trigger timestamp messages'
    )

    debounce_time_arg = DeclareLaunchArgument(
        'debounce_time_us',
        default_value='50',
        description='Trigger GPIO debounce time in microseconds'
    )

    # PPS GPIO (1 Hz GPS PPS, gpiochip0:96)
    pps_gpio_chip_arg = DeclareLaunchArgument(
        'pps_gpio_chip',
        default_value='/dev/gpiochip0',
        description='GPIO chip for GPS PPS signal'
    )

    pps_gpio_line_arg = DeclareLaunchArgument(
        'pps_gpio_line',
        default_value='96',
        description='GPIO line for GPS PPS signal'
    )

    pps_frame_id_arg = DeclareLaunchArgument(
        'pps_frame_id',
        default_value='gq7_pps',
        description='Frame ID for /gq7/ext/time TimeReference messages'
    )

    pps_debounce_arg = DeclareLaunchArgument(
        'pps_debounce_time_us',
        default_value='0',
        description='PPS GPIO debounce time in microseconds'
    )
    
    freq_window_arg = DeclareLaunchArgument(
        'freq_window_size',
        default_value='100',
        description='Number of samples for frequency calculation'
    )
    
    running_freq_window_arg = DeclareLaunchArgument(
        'running_freq_window_size',
        default_value='100',
        description='Number of samples for running frequency in diagnostics'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Debug mode: auto-stop after 1 second'
    )

    debug_duration_arg = DeclareLaunchArgument(
        'debug_duration_s',
        default_value='0.2',
        description='Debug auto-stop duration in seconds'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_continuous_diagnostics',
        default_value='false',
        description='Enable continuous diagnostic publishing (queries Teensy periodically)'
    )
    
    diagnostic_period_arg = DeclareLaunchArgument(
        'diagnostic_period',
        default_value='1.0',
        description='Period for continuous diagnostics in seconds (if enabled)'
    )

    stop_capture_grace_arg = DeclareLaunchArgument(
        'stop_capture_grace_ms',
        default_value='150.0',
        description='Keep GPIO capture alive briefly after STOP (ms)'
    )

    stop_empty_polls_arg = DeclareLaunchArgument(
        'stop_empty_polls_before_exit',
        default_value='3',
        description='Required empty GPIO polls after STOP before monitor thread exits'
    )

    start_drain_arg = DeclareLaunchArgument(
        'start_drain_ms',
        default_value='50.0',
        description='Drain stale GPIO edge events before START (ms)'
    )
    
    # Node
    teensy_node = Node(
        package='bag_record_pid',
        executable='teensy_combined',
        name='teensy_combined',
        output='screen',
        parameters=[{
            'serial_port':                  LaunchConfiguration('serial_port'),
            'baud_rate':                    LaunchConfiguration('baud_rate'),
            'trigger_gpio_chip':            LaunchConfiguration('trigger_gpio_chip'),
            'trigger_gpio_line':            LaunchConfiguration('trigger_gpio_line'),
            'trigger_frame_id':             LaunchConfiguration('trigger_frame_id'),
            'debounce_time_us':             LaunchConfiguration('debounce_time_us'),
            'pps_gpio_chip':                LaunchConfiguration('pps_gpio_chip'),
            'pps_gpio_line':                LaunchConfiguration('pps_gpio_line'),
            'pps_frame_id':                 LaunchConfiguration('pps_frame_id'),
            'pps_debounce_time_us':         LaunchConfiguration('pps_debounce_time_us'),
            'freq_window_size':             LaunchConfiguration('freq_window_size'),
            'running_freq_window_size':     LaunchConfiguration('running_freq_window_size'),
            'debug_mode':                   LaunchConfiguration('debug_mode'),
            'debug_duration_s':             LaunchConfiguration('debug_duration_s'),
            'enable_continuous_diagnostics': LaunchConfiguration('enable_continuous_diagnostics'),
            'diagnostic_period':            LaunchConfiguration('diagnostic_period'),
            'stop_capture_grace_ms':        LaunchConfiguration('stop_capture_grace_ms'),
            'stop_empty_polls_before_exit': LaunchConfiguration('stop_empty_polls_before_exit'),
            'start_drain_ms':               LaunchConfiguration('start_drain_ms'),
        }]
    )


    container_1 = ComposableNodeContainer(
        name="teensy_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=make_recorder_nodes(keys=[f"trigger"]) + make_recorder_nodes(keys=[f"tf"]),
        output="screen",
    )

    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        gpio_chip_arg,
        gpio_line_arg,
        frame_id_arg,
        debounce_time_arg,
        pps_gpio_chip_arg,
        pps_gpio_line_arg,
        pps_frame_id_arg,
        pps_debounce_arg,
        freq_window_arg,
        running_freq_window_arg,
        debug_mode_arg,
        debug_duration_arg,
        enable_diagnostics_arg,
        diagnostic_period_arg,
        stop_capture_grace_arg,
        stop_empty_polls_arg,
        start_drain_arg,
        teensy_node,
        container_1
    ])
