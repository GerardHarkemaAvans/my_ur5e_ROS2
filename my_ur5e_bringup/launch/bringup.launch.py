import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    moveit_group_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_ur5e_moveit_config'), 'launch'),
         '/move_group.launch.py'])
      )


    rviz_config = os.path.join(
        get_package_share_directory('my_ur5e_bringup'),
        'rviz',
        'rviz_config.rviz'
     )
     
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
     
    return LaunchDescription([
        moveit_group_node,
        rviz_node
    ])
