import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    pkg_dir = get_package_share_directory('stalkerv2')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    detection_2d_topic = '/yolo_detections_2d'
    
    # Define arguments
    cam_arg = DeclareLaunchArgument(
        'camera',
        default_value='/depth/image_raw',
        description='Camera topic to use'
    )
    
    # Use a proper conditional for remappings
    camera_depth = PythonExpression([
        "'", '/rgbd_camera/depth_image', "' if '", LaunchConfiguration('camera'), "' == 'simulator' else '",
        LaunchConfiguration('camera'), "'"
    ])
    
    camera_depth_info = PythonExpression([
        "'", '/rgbd_camera/camera_info', "' if '", LaunchConfiguration('camera'), "' == 'simulator' else '",
        '/depth/camera_info', "'"
    ])
    
    return LaunchDescription([
        cam_arg,
        Node(
            package='camera',
            executable='detection_2d_to_3d_depth',
            output='screen',
            parameters=[param_file],
            remappings=[
                ('input_depth', camera_depth),
                ('input_detection_2d', detection_2d_topic),
                ('camera_info', camera_depth_info),
                ('output_detection_3d', '/coordinates/attractive'),
            ]
        )
    ])