from pathlib import Path
from shlex import quote

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = Path(get_package_share_directory('camera_ros'))
    camera_launch = camera_share / 'launch' / 'camera.launch.py'
    parking_config = Path(get_package_share_directory('parking')) / 'config' / 'parking_sim.yaml'
    venv_python = Path(__file__).resolve().parent / '.venv' / 'bin' / 'python'
    yolo_prefix = quote(str(venv_python))

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(camera_launch))),
        Node(
            package='driving',
            executable='driving',
            prefix=yolo_prefix,
            output='screen'
        ),
        # Node(
        #     package='driving',
        #     executable='driving_debug',
        #     output='screen'
        # ),
        Node(
            package='mission_model',
            executable='detection',
            prefix=yolo_prefix,
            output='screen'
        ),
        Node(
            package='construction',
            executable='detection',
            output='screen'
        ),
        #Node(
        #    package='level_crossing',
        #    executable='detection',
        #    output='screen'
        #),
        #Node(
        #   package='parking',
        #   executable='detection',
        #   parameters=[str(parking_config)],
        #   output='screen'
        #),
        Node(
            package='traffic_light',
            executable='detection',
            output='screen'
        ),
        Node(
            package='tunnel',
            executable='detection',
            output='screen'
        ),

    ])
