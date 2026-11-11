"""Mission sign detection -> /mission_mode.

  ros2 launch mission_model mission_model.launch.py              # real robot: front camera on /camera2
  ros2 launch mission_model mission_model.launch.py sim:=true    # Gazebo: /front_camera/image_raw -> /camera2

The default .onnx model needs only onnxruntime (on the Jetson: onnxruntime-gpu for JetPack 5);
a .pt model needs ultralytics. By default the node uses the installed detection script's
venv shebang. Set `python` to override its interpreter explicitly. The default device is CUDA.
"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _detection(context):
    python = LaunchConfiguration('python').perform(context)
    script = os.path.join(get_package_prefix('mission_model'), 'lib', 'mission_model', 'detection')
    sim = LaunchConfiguration('sim').perform(context)
    args = ['--ros-args', '-p', f'use_sim_time:={sim}',
            '-p', f"device:='{LaunchConfiguration('device').perform(context)}'",
            '-p', f"show:={LaunchConfiguration('show').perform(context)}"]
    command = [python, '-u', script, *args] if python else [script, *args]
    return [ExecuteProcess(cmd=command, output='screen', name='mission_detection')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true: convert the Gazebo front camera to /camera2'),
        DeclareLaunchArgument('python', default_value='', description='python with onnxruntime (.onnx) or ultralytics (.pt)'),
        DeclareLaunchArgument('device', default_value='cuda', description="'cuda' default, '' auto, 'cpu', '0', 'trt' (trt/cuda: .onnx only)"),
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
