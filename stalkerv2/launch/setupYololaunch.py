import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    pkg_dir = get_package_share_directory('stalkerv2')
    model_path = os.path.join(pkg_dir, 'models')

    # Define arguments
    cam_arg = DeclareLaunchArgument(
        'camera',
        default_value='/color/image_raw',
        description='Camera topic to use')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=model_path + '/yolov8m-pose.pt',
        description='Model to use',
    )

    return LaunchDescription(
        [
            cam_arg,
            model_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('stalkerv2'),
                        'launch',
                        'yoloNodelaunch.py',
                    )
                ),
                launch_arguments={
                    'camera': LaunchConfiguration('camera'),
                    'model': LaunchConfiguration('model'),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('stalkerv2'),
                        'launch',
                        'yoloDetectionlaunch.py',
                    )
                )
            ),
        ]
    )
