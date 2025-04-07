import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """Generate launch description for YOLOv8 node."""  
    pkg_dir = get_package_share_directory('stalkerv2')
    model_path = os.path.join(pkg_dir, 'models')
    
    # Define arguments
    cam_arg = DeclareLaunchArgument(
        'camera',
        default_value='/color/image_raw',
        description='Camera topic to use'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=model_path + '/yolov8m-pose.pt',
        description='Model to use'
    )
    
    # Use PythonExpression for camera topic selection
    camera_topic = PythonExpression([
        "'/rgb/image_raw' if '", LaunchConfiguration('camera'), "' == 'xtion' else ",
        "'/rgbd_camera/image' if '", LaunchConfiguration('camera'), "' == 'simulator' else ",
        "'/image_raw' if '", LaunchConfiguration('camera'), "' == 'laptop' else '",
        LaunchConfiguration('camera'), "'"
    ])
    
    # Use PythonExpression for model selection - now with proper quotes
    model_file = PythonExpression([
        "'" + model_path + "/yolov8m.pt' if '", LaunchConfiguration('model'), "' == 'v8' else ",
        "'" + model_path + "/yolov8m-seg.pt' if '", LaunchConfiguration('model'), "' == 'seg' else '",
        LaunchConfiguration('model'), "'"
    ])
    
    return LaunchDescription([
        cam_arg,
        model_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('yolo_bringup'),
                    'launch',
                    'yolo.launch.py',
                )
            ),
            launch_arguments={
                'model': model_file,
                'input_image_topic': camera_topic,
            }.items(),
        )
    ])