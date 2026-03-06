import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   # *** REMEMBER: Change this to your actual world file ***
   world_file = os.path.join(get_package_share_directory('rbsf_mvsim'),'rbsf_mvsim','rbsf_forzaeth.world.xml')
   rviz_config_file = os.path.join(get_package_share_directory('rbsf_mvsim'),'rbsf_mvsim','rbsf_forzaeth_ro2.rviz')

   headless = 'False'
   do_fake_localization = 'True'
   publish_tf_odom2baselink = 'True'
   force_publish_vehicle_namespace = 'False'
   use_rviz = 'False'

   # Create LaunchDescription
   ld = LaunchDescription()

   # Add actions to LaunchDescription
   ld.add_action(IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mvsim'), 'launch', 'launch_world.launch.py')
      ),
      launch_arguments={
            'world_file': world_file,
            'headless': headless,
            'do_fake_localization': do_fake_localization,
            'publish_tf_odom2baselink': publish_tf_odom2baselink,
            'force_publish_vehicle_namespace': force_publish_vehicle_namespace,
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config_file
      }.items()
   ))

   return ld