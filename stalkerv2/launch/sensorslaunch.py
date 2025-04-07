import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Define arguments
    cam_arg = DeclareLaunchArgument(
        'camera',
        default_value='/depth/image_raw',
        description='Camera topic to use')

    laser_arg = DeclareLaunchArgument(
        'laser', default_value='/scan', description='Laser to use'
    )

    return LaunchDescription(
        [
            cam_arg,
            laser_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('stalkerv2'),
                        'launch',
                        'detection3d_hsv.launch.py',
                    )
                ),
                launch_arguments={
                    'camera': LaunchConfiguration('camera'),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('stalkerv2'),
                        'launch',
                        'laser.launch.py',
                    )
                ),
                launch_arguments={
                    'laser': LaunchConfiguration('laser'),
                }.items(),
            ),
        ]
    )
