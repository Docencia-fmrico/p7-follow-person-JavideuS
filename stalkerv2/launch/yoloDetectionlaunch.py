from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for yolo detection node."""
    input_topic = '/yolo/detections'
    output_topic = '/2d_yolo_detections'
    
    return LaunchDescription([
        Node(
            package='camera',
            executable='yolo_detection',
            output='screen',
            remappings=[
                ('input_detection', input_topic),
                ('output_detection_2d', output_topic),
            ])
    ])
