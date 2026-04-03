from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bag_file_arg = DeclareLaunchArgument(
        'rosbag_file',
        description='Path to the rosbag file to play')
    
    lector_node = Node(
            package='rbsf_mvsim',
            namespace='rbsf',
            name='lidar_log',
            executable='lidar_logger'
    )
    
    play_bag = ExecuteProcess(
        cmd=['ros2','bag','play',
        LaunchConfiguration('rosbag_file')]
    )

    return LaunchDescription([
        bag_file_arg,
        lector_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action = lector_node,
                on_start=[play_bag])
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action= play_bag,
                on_exit= [EmitEvent(event= Shutdown())])
        )
    ])