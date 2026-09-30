"""Launch file for centerline manual operation."""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import os
import math
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

def parse_world_and_create_tf(context, *args, **kwargs):
    """
    Parses the world XML file and creates a static transform publisher.

    Args:
        context: The launch context.
        *args: Variable length argument list.
        **kwargs: Arbitrary keyword arguments.

    Returns:
        list: A list containing the Node action for the static transform publisher.
    """
    world_path = context.perform_substitution(LaunchConfiguration('world_file'))
    x, y, yaw_rad = '0.0', '0.0', '0.0'
    
    try:
        if os.path.exists(world_path):
            tree = ET.parse(world_path)
            root = tree.getroot()
            init_pose_el = root.find(".//vehicle/init_pose")
            
            if init_pose_el is not None and init_pose_el.text:
                pose_parts = init_pose_el.text.strip().split()
                if len(pose_parts) >= 3:
                    x = pose_parts[0]
                    y = pose_parts[1]
                    yaw_rad = str(math.radians(float(pose_parts[2])))
                    print(f"\n[INFO] Successfully generated TF -> X: {x}, Y: {y}, Yaw: {yaw_rad} rad\n")
    except Exception as e:
        print(f"\n[ERROR] Failed to read XML: {e}\n")

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=[
                x, y, '0.0', 
                yaw_rad, '0.0', '0.0', 
                'global_map', 'map'   # <-- HERE: We link the new frame with the old one
            ]
        )
    ]

def generate_launch_description():
    """
    Generates the launch description for the centerline manual node.

    Returns:
        LaunchDescription: The complete launch description with declared arguments and nodes.
    """
    pkg_share = get_package_share_directory('rbsf_mvsim')

    world_file_launch_arg = DeclareLaunchArgument(
        "world_file",
        default_value=os.path.join(pkg_share, 'rbsf_mvsim', 'f1tenth_catalunya.world.xml'),
        description='Path to the *.world.xml file to load')

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_share, 'rbsf_mvsim', 'rbsf_f1tenth_ros2.rviz'),
        description='Path to the configuration file for rviz'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_share, 'rbsf_mvsim', 'Catalunya_map.yaml'),
        description='Path to the map YAML file'
    )

    return LaunchDescription([
        world_file_launch_arg,
        rviz_config_file_arg,
        map_file_arg,
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
                "headless": False,
                "do_fake_location": False,
                "publish_tf_odom2baselink": True,
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_file'),
                'frame_id': 'global_map'  # <-- HERE: We inject the new base frame
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),
        Node(
            package='rbsf_mvsim',
            executable='path_publisher',
            name='path_publisher',
        ),
        Node(
            package='rbsf_mvsim',
            executable='waypoint_logger',
            name='waypoint_logger',
        ),
        OpaqueFunction(function=parse_world_and_create_tf),
    ])