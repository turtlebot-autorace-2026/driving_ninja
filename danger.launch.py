from pathlib import Path
from shlex import quote

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    parking_config = Path(get_package_share_directory('parking')) / 'config' / 'parking_sim.yaml'

    return LaunchDescription([
        Node(
           package='parking',
           executable='detection',
           parameters=[str(parking_config)],
           output='screen'
        ),
        
        #Node(
        #   package='level_crossing',
        #   executable='detection',
        #   output='screen'
        #),
        
    ])
