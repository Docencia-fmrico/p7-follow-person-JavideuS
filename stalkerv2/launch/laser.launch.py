import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('stalker')
    param_file = os.path.join(pkg_dir, 'config', 'paramsLaser.yaml')

    laser_arg = DeclareLaunchArgument(
        'laser',
        default_value='/scan',
        description='Laser to use'
    )

    laser_topic = PythonExpression([
        "'/scan_raw' if '", LaunchConfiguration('laser'), "' == 'simulator' else '",
        LaunchConfiguration('laser'), "'"
    ])

    return LaunchDescription([
        laser_arg,
        Node(
            package='stalker',
            executable='obstacle_detector',
            output='screen',
            parameters=[param_file],
            remappings=[('input_scan', laser_topic)]
        ),
    ])
