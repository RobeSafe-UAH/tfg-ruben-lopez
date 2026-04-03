from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
        Launch file description: Orchest a bunch of nodes and its configuration.
        Import from 'general launch' from ROS2 and adapted due the project 
        'Diseño de un robot autónomo de carreras con ROS2'.
    """

    # args that can be set from the command line or a default will be used
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file",
        default_value= os.path.join(get_package_share_directory('rbsf_mvsim'),'rbsf_mvsim','rbsf_f1tenth.world.xml'),
        description='Path to the *.world.xml file to load')


    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(get_package_share_directory('rbsf_mvsim'),'rbsf_mvsim','rbsf_f1tenth_ros2.rviz'),
        description='Path to the configuration file for rviz'
    )

    headless_launch_arg = DeclareLaunchArgument(
        "headless", default_value='False')

    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization",
        default_value='True',
        description='publish fake identity tf "map" -> "odom"'
    )

    publish_tf_odom2baselink_arg = DeclareLaunchArgument(
        "publish_tf_odom2baselink",
        default_value='True',
        description='publish tf "odom" -> "base_link"'
    )

    force_publish_vehicle_namespace_arg = DeclareLaunchArgument(
        "force_publish_vehicle_namespace",
        default_value='True',
        description='Use vehicle name namespace even if there is only one vehicle'
    )

    DBSCAN_parameters_arg = DeclareLaunchArgument(
        'DBSCAN_parameters',
        default_value='',
        description=''
    )

    norfair_parameters_arg = DeclareLaunchArgument(
        'norfair_parameters',
        default_value='',
        description=''
    )

    return LaunchDescription([
        world_file_launch_arg,
        rviz_config_file_arg,
        headless_launch_arg,
        do_fake_localization_arg,
        publish_tf_odom2baselink_arg,
        force_publish_vehicle_namespace_arg,

        # Declaration of the 'rviz2' node, remapping the '/tf/' and '/tf_static'
        # topics to their respective topics prefixed with the namespace of
        # robot 1 ('/r1'). If no configuration file is detected, a new one
        # is opened.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            remappings=[
                ('/tf','/r1/tf'),
                ('/tf_static','/r1/tf_static')],
            arguments=[] if LaunchConfiguration('rviz_config_file') == '' 
                else ['-d', LaunchConfiguration('rviz_config_file')]
        ),

        Node(
            package='mvsim',
            executable='mvsim_node',
            name='mvsim',
            output='screen',
            parameters=[
                {
                    "world_file": LaunchConfiguration('world_file'),
                    "headless": LaunchConfiguration('headless'),
                    "do_fake_localization": LaunchConfiguration('do_fake_localization'),
                    "publish_tf_odom2baselink": LaunchConfiguration('publish_tf_odom2baselink'),
                    "force_publish_vehicle_namespace": LaunchConfiguration('force_publish_vehicle_namespace'),
                }]
        ),

        # Declaration of the 'perception' node, which perceives the environment,
        # detects obstacles, filters them, and tracks them. It allows you to
        # inject the necessary parameters for the detector and tracker.
        Node(
            package='rbsf_mvsim',
            executable='perception',
            name='perception'
        )
    ])