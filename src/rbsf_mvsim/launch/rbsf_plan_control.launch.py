"""Launch file for planning and control nodes execution."""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generates the launch description for planning and control.

    Returns:
        LaunchDescription: The configured launch description object.
    """
    pkg_share = get_package_share_directory('rbsf_mvsim')

    world_file_launch_arg = DeclareLaunchArgument("world_file", default_value=os.path.join(pkg_share, 'rbsf_mvsim', 'f1tenth_catalunya.world.xml'))
    rviz_config_file_arg = DeclareLaunchArgument('rviz_config_file', default_value=os.path.join(pkg_share, 'rbsf_mvsim', 'rbsf_f1tenth_ros2.rviz'))
    map_file_arg = DeclareLaunchArgument('map_file', default_value=os.path.join(pkg_share, 'rbsf_mvsim', 'Catalunya_map.yaml'))

    return LaunchDescription([
        world_file_launch_arg, rviz_config_file_arg, map_file_arg,

        Node(package='rviz2', executable='rviz2', name='rviz', arguments=[] if LaunchConfiguration('rviz_config_file') == '' else ['-d', LaunchConfiguration('rviz_config_file')]),

        Node(
            package='mvsim',
            executable='mvsim_node',
            name='mvsim',
            output='screen',
            parameters=[{
                "world_file": LaunchConfiguration('world_file'),
                "headless": False,
                # Disable native TFs of the simulator.
                # Visualization will be built externally with absolute coordinates.
                "do_fake_location": False,
                "publish_tf_odom2baselink": False, 
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map_file'), 'frame_id': 'map'}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'autostart': True}, {'node_names': ['map_server']}]
        ),

        Node(package='rbsf_mvsim', executable='path_publisher', name='path_publisher'),
        Node(package='rbsf_mvsim', executable='offline_planner_node', name='offline_planner_node'),

        Node(
            package='rbsf_mvsim',
            executable='controller_node',
            name='controller_node',
            remappings=[('/odom', '/base_pose_ground_truth')]
        ),

        Node(package='rbsf_mvsim', executable='ackermann_to_twist', name='ackermann_to_twist_bridge', parameters=[{'wheelbase': 0.33}]),

        Node(
            package='rbsf_mvsim',
            executable='metrics_evaluator',
            name='metrics_evaluator_node',
            output='screen',
            remappings=[('/odom', '/base_pose_ground_truth')]
        ),
    ])