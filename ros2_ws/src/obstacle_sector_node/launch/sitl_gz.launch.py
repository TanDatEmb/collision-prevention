"""Gazebo SITL: bridge the 2D lidar scan and the clock to ROS and run obstacle_sector_node on
simulated time (ADR 0003: use_sim_time, and UXRCE_DDS_SYNCT=0 on the PX4 side).

`platform_config` has no default: a platform YAML with approved values is a decision, not a guess.
`gz_scan_topic` is the Gazebo topic of the lidar of the `x500_lidar_2d` model of PX4 SITL; change it
when the world or the model instance has another name.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gz_scan_topic = LaunchConfiguration('gz_scan_topic')
    return LaunchDescription([
        DeclareLaunchArgument(
            'platform_config',
            description='Path of the platform YAML (required, no default).'),
        DeclareLaunchArgument(
            'gz_scan_topic',
            default_value='/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
            description='Gazebo topic of the planar lidar scan.'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_scan_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                [gz_scan_topic, TextSubstitution(text='@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')],
            ],
            remappings=[(gz_scan_topic, 'scan')],
            output='screen'),
        Node(
            package='obstacle_sector_node',
            executable='obstacle_sector_node',
            parameters=[{
                'use_sim_time': True,
                'platform_config': LaunchConfiguration('platform_config'),
                'scan_topic': 'scan',
            }],
            output='screen'),
    ])
