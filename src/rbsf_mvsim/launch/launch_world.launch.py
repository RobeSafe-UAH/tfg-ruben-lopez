"""Generic ROS2 launch file for world execution."""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Generates the launch description for the world execution.

    Returns:
        LaunchDescription: The configured launch description.
    """
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file", description='Path to the *.world.xml file to load')

    headless_launch_arg = DeclareLaunchArgument(
        "headless", default_value='False')

    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization", default_value='True', description='Publish fake identity tf "map" -> "odom"')

    publish_tf_odom2baselink_arg = DeclareLaunchArgument(
        "publish_tf_odom2baselink", default_value='True', description='Publish tf "odom" -> "base_link"')

    force_publish_vehicle_namespace_arg = DeclareLaunchArgument(
        "force_publish_vehicle_namespace", default_value='False',
        description='Use vehicle name namespace even if there is only one vehicle')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Whether to launch RViz2')

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file', default_value='',
        description='If use_rviz:="True", the configuration file for rviz')

    mvsim_node = Node(
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
    )

    detector_node = Node(
        package='rbsf_mvsim',
        executable='detector_w_track',
        name='detector'
    )

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        remappings=[
            ('/tf','/r1/tf'),
            ('/tf_static','/r1/tf_static')
        ],
        arguments=[] if LaunchConfiguration('rviz_config_file') == '' else [
                '-d', LaunchConfiguration('rviz_config_file')]
    )

    return LaunchDescription([
        world_file_launch_arg,
        headless_launch_arg,
        do_fake_localization_arg,
        publish_tf_odom2baselink_arg,
        force_publish_vehicle_namespace_arg,
        use_rviz_arg,
        rviz_config_file_arg,
        mvsim_node,
        detector_node,
        rviz2_node
    ])
