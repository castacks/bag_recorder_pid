"""
Launch file for GQ7 PPS Time Sync Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    gpio_chip_arg = DeclareLaunchArgument(
        'gpio_chip',
        default_value='/dev/gpiochip0',
        description='GPIO chip device path (same pin as Teensy trigger)')

    gpio_line_arg = DeclareLaunchArgument(
        'gpio_line',
        default_value='96',
        description='GPIO line offset for PPS signal')

    debounce_arg = DeclareLaunchArgument(
        'debounce_time_us',
        default_value='0',
        description='GPIO debounce time in microseconds')

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gq7_pps',
        description='Frame ID for TimeReference messages')

    node = Node(
        package='bag_record_pid',
        executable='gq7_pps_time_sync',
        name='gq7_pps_time_sync',
        output='screen',
        parameters=[{
            'gpio_chip':        LaunchConfiguration('gpio_chip'),
            'gpio_line':        LaunchConfiguration('gpio_line'),
            'debounce_time_us': LaunchConfiguration('debounce_time_us'),
            'frame_id':         LaunchConfiguration('frame_id'),
        }]
    )

    return LaunchDescription([
        gpio_chip_arg,
        gpio_line_arg,
        debounce_arg,
        frame_id_arg,
        node,
    ])
