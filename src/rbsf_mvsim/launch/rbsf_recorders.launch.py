"""Launch file to play rosbags and log LiDAR data."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown

def generate_launch_description():
    """
    Generates the launch description to play rosbags and record lidar info.

    Returns:
        LaunchDescription: The configured launch description object.
    """
    bag_file_arg = DeclareLaunchArgument(
        'rosbag_file',
        description='Path to the rosbag file to play')
    
    reader_node = Node(
        package='rbsf_mvsim',
        namespace='rbsf',
        name='lidar_log',
        executable='lidar_logger'
    )
    
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_file')]
    )

    return LaunchDescription([
        bag_file_arg,
        reader_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=reader_node,
                on_start=[play_bag])
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=play_bag,
                on_exit=[EmitEvent(event=Shutdown())])
        )
    ])