"""Launch file for orchestrating nodes in a real Traxxas environment."""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generates the launch description for real Traxxas execution.
    Imported from 'general launch' from ROS2 and adapted for the project
    'Design of an autonomous racing robot with ROS2'.

    Returns:
        LaunchDescription: The configured launch description object.
    """
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(get_package_share_directory('rbsf_mvsim'), 'rbsf_mvsim', 'rbsf_traxxas_real_ros2.rviz'),
        description='Path to the configuration file for rviz'
    )

    bag_file_arg = DeclareLaunchArgument(
        'rosbag_file',
        default_value=os.path.join('/', 'sim_ws', 'src', 'rosbags', 'traxxas_real_5'),
        description='Path to the rosbag file to play'
    )
        
    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization",
        default_value='True',
        description='Publish fake identity tf "map" -> "odom"'
    )

    publish_tf_odom2baselink_arg = DeclareLaunchArgument(
        "publish_tf_odom2baselink",
        default_value='True',
        description='Publish tf "odom" -> "base_link"'
    )

    force_publish_vehicle_namespace_arg = DeclareLaunchArgument(
        "force_publish_vehicle_namespace",
        default_value='True',
        description='Use vehicle name namespace even if there is only one vehicle'
    )

    eps_arg = DeclareLaunchArgument(
        'eps',
        default_value="0.3",
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

    laser_topic_arg = DeclareLaunchArgument(
        'laser_topic',
        default_value="/scan",
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

    max_obs_size_arg = DeclareLaunchArgument(
        'max_obs_size',
        default_value="0.55",
        description='Maximum car length to consider an obstacle'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("rbsf_mvsim"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    detector_node = Node(
        package='rbsf_mvsim',
        executable='visualization_detector_node',
        name='detector',
        parameters=[{
            'eps': LaunchConfiguration('eps'),
            'min_samples': LaunchConfiguration('min_samples'),
            'min_range': LaunchConfiguration('min_range'),
            'max_range': LaunchConfiguration('max_range'),
            'max_obs_size': LaunchConfiguration('max_obs_size'),
            'laser_topic': LaunchConfiguration('laser_topic')
        }]
    )
    
    tracker_node = Node(
        package='rbsf_mvsim',
        executable='visualization_tracker_node',
        name='tracker',
        parameters=[{
            'distance_function': LaunchConfiguration('distance_function'),
            'distance_threshold': LaunchConfiguration('distance_threshold'),
            'hit_counter_max': LaunchConfiguration('hit_counter_max'),
            'initialization_delay': LaunchConfiguration('initialization_delay'),
            'max_obs_size': LaunchConfiguration('max_obs_size')
        }]
    )

    return LaunchDescription([
        rviz_config_file_arg,
        bag_file_arg,
        do_fake_localization_arg,
        publish_tf_odom2baselink_arg,
        force_publish_vehicle_namespace_arg,
        eps_arg,
        min_samples_arg,
        max_range_arg,
        min_range_arg,
        max_obs_size_arg,
        laser_topic_arg,
        distance_function_arg,
        distance_threshold_arg,
        hit_counter_max_arg,
        initialization_delay_arg,
        detector_node,
        tracker_node,
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
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file')
            ]
        )
    ])