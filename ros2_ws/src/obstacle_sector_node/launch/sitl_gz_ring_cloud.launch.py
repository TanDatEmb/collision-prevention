"""Gazebo SITL with the bench multi-ring lidar: bridge the point cloud and the clock to ROS and run
obstacle_sector_node on simulated time with the `ring_cloud` source (issue #138, ADR 0022 decision 4).

`platform_config` has no default: a platform YAML with approved values is a decision, not a guess.
`layout_config` is the layout of the simulated sensor (tools/scenarios/multi_ring_lidar_layout.yaml); it
has no default either, so the layout and the SDF cannot drift apart unseen.
`gz_cloud_topic` is the Gazebo topic of the cloud of the `x500_lidar_multi_ring` model instance of PX4 SITL
(tools/scenarios/models); change it when the world or the model instance has another name.

The node levels the rays with the mount and the FMU pose from `/fmu/out/vehicle_odometry` (#179), so PX4
SITL and the uXRCE-DDS agent must run; without odometry the frames are dropped and no sector is clear.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gz_cloud_topic = LaunchConfiguration('gz_cloud_topic')
    return LaunchDescription([
        DeclareLaunchArgument(
            'platform_config',
            description='Path of the platform YAML (required, no default).'),
        DeclareLaunchArgument(
            'layout_config',
            description='Path of the ring layout YAML (required, no default).'),
        DeclareLaunchArgument(
            'gz_cloud_topic',
            default_value='/world/default/model/x500_lidar_multi_ring_0/link/lidar_link/sensor/'
                          'lidar_multi_ring/scan/points',
            description='Gazebo topic of the multi-ring point cloud.'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_cloud_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                [gz_cloud_topic,
                 TextSubstitution(text='@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')],
            ],
            remappings=[(gz_cloud_topic, 'points')],
            output='screen'),
        Node(
            package='obstacle_sector_node',
            executable='obstacle_sector_node',
            parameters=[
                LaunchConfiguration('layout_config'),
                {
                    'use_sim_time': True,
                    'platform_config': LaunchConfiguration('platform_config'),
                    'cloud_topic': 'points',
                },
            ],
            output='screen'),
    ])
