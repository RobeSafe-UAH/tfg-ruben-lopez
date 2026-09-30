"""Launch file to orchestrate nodes and their configuration for Traxxas simulation."""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generates the launch description for the Traxxas simulation.
    Imported from 'general launch' from ROS2 and adapted for the project
    'Design of an autonomous racing robot with ROS2'.

    Returns:
        LaunchDescription: The configured launch description object.
    """
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file",
        default_value=os.path.join(get_package_share_directory('rbsf_mvsim'), 'rbsf_mvsim', 'rbsf_f1tenth_obs.world.xml'),
        description='Path to the *.world.xml file to load')

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(get_package_share_directory('rbsf_mvsim'), 'rbsf_mvsim', 'rbsf_f1tenth_ros2.rviz'),
        description='Path to the configuration file for rviz'
    )

    headless_launch_arg = DeclareLaunchArgument(
        "headless", default_value='False'
    )

    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization",
        default_value='False',
        description='Publish fake identity tf "map" -> "odom"'
    )

    publish_tf_odom2baselink_arg = DeclareLaunchArgument(
        "publish_tf_odom2baselink",
        default_value='True',
        description='Publish tf "odom" -> "base_link"'
    )

    force_publish_vehicle_namespace_arg = DeclareLaunchArgument(
        "force_publish_vehicle_namespace",
        default_value='False',
        description='Use vehicle name namespace even if there is only one vehicle'
    )

    eps_arg = DeclareLaunchArgument(
        'eps',
        default_value="0.4",
        description='DBSCAN parameter: Maximum distance to consider two points adjacent'
    )

    min_samples_arg = DeclareLaunchArgument(
        'min_samples',
        default_value="10",
        description='DBSCAN parameter: Minimum quantity of samples to be considered a cluster'
    )

    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value="0.1",
        description='Minimum working distance of LiDAR'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value="30.0",
        description='Maximum working distance of LiDAR'
    )

    max_obs_size_arg = DeclareLaunchArgument(
        'max_obs_size',
        default_value="0.55",
        description='Maximum car length to consider an obstacle'
    )

    laser_topic_arg = DeclareLaunchArgument(
        'laser_topic',
        default_value="/laser1",
        description='Topic where the detector extracts the LiDAR information'
    )

    distance_function_arg = DeclareLaunchArgument(
        'distance_function',
        default_value="euclidean",
        description='Distance function parameter of the tracker'
    )

    distance_threshold_arg = DeclareLaunchArgument(
        'distance_threshold',
        default_value="0.5",
        description='Distance threshold of the tracker'  
    )

    hit_counter_max_arg = DeclareLaunchArgument(
        'hit_counter_max',
        default_value="100",
        description='Maximum hits of a track without seeing it'
    )

    initialization_delay_arg = DeclareLaunchArgument(
        'initialization_delay',
        default_value="3",
        description='Minimum number of hits to be considered a track'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("rbsf_mvsim"),
                                   'config', 'mapper_params_sim.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    return LaunchDescription([
        world_file_launch_arg,
        rviz_config_file_arg,
        headless_launch_arg,
        do_fake_localization_arg,
        publish_tf_odom2baselink_arg,
        force_publish_vehicle_namespace_arg,
        eps_arg,
        min_samples_arg,
        min_range_arg,
        max_range_arg,
        max_obs_size_arg,
        laser_topic_arg,
        distance_function_arg,
        distance_threshold_arg,
        hit_counter_max_arg,
        initialization_delay_arg,
        slam_params_file_arg,
        use_sim_time_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=[] if LaunchConfiguration('rviz_config_file') == '' 
                else ['-d', LaunchConfiguration('rviz_config_file')]
        ),
        Node(
            package='mvsim',
            executable='mvsim_node',
            name='mvsim',
            output='screen',
            parameters=[{
                "world_file": LaunchConfiguration('world_file'),
                "headless": LaunchConfiguration('headless'),
                "do_fake_localization": LaunchConfiguration('do_fake_localization'),
                "publish_tf_odom2baselink": LaunchConfiguration('publish_tf_odom2baselink'),
                "force_publish_vehicle_namespace": LaunchConfiguration('force_publish_vehicle_namespace'),
            }]
        )
    ])