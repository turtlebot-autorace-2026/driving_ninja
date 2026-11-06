"""Mission sign detection -> /mission_mode.

  ros2 launch mission_model mission_model.launch.py              # real robot: front camera on /camera2
  ros2 launch mission_model mission_model.launch.py sim:=true    # Gazebo: /front_camera/image_raw -> /camera2

ultralytics is needed, so the node runs with `python` (default: ~/autorace/yolo_venv/bin/python
if it exists, otherwise python3).
"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VENV_PYTHON = os.path.expanduser('~/autorace/yolo_venv/bin/python')


def _detection(context):
    python = LaunchConfiguration('python').perform(context) or \
        (VENV_PYTHON if os.path.exists(VENV_PYTHON) else 'python3')
    script = os.path.join(get_package_prefix('mission_model'), 'lib', 'mission_model', 'detection')
    sim = LaunchConfiguration('sim').perform(context)
    args = ['--ros-args', '-p', f'use_sim_time:={sim}',
            '-p', f"device:='{LaunchConfiguration('device').perform(context)}'",
            '-p', f"show:={LaunchConfiguration('show').perform(context)}"]
    return [ExecuteProcess(cmd=[python, '-u', script, *args], output='screen', name='mission_detection')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true: convert the Gazebo front camera to /camera2'),
        DeclareLaunchArgument('python', default_value='', description='python with ultralytics'),
        DeclareLaunchArgument('device', default_value='', description="'' auto, 'cpu', '0'"),
        DeclareLaunchArgument('show', default_value='false', description='cv2 debug window'),
        Node(
            package='camera_ros',
            executable='sim_camera',
            name='front_camera_publisher',
            parameters=[{'input_topic': '/front_camera/image_raw', 'output_topic': '/camera2'}],
            condition=IfCondition(LaunchConfiguration('sim')),
        ),
        OpaqueFunction(function=_detection),
    ])
