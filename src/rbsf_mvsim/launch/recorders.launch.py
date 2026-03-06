from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
import os

BAG_FILE= os.path.join('/','sim_ws','src','rosbag2_2026_03_03-14_21_16')
def generate_launch_description():

    lector_node = Node(
            package='rbsf_mvsim',
            namespace='rbsf',
            name='lidar_log',
            executable='lidar_logger'
        )
    
    play_bag = ExecuteProcess(
        cmd=['ros2','bag','play',BAG_FILE]
    )

    return LaunchDescription([
        lector_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action = lector_node,
                on_start=[play_bag]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action= play_bag,
                on_exit= [EmitEvent(event= Shutdown())]
            )
        )
    ])