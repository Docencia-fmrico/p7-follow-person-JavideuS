from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = '/yolo/detections'
    output_topic = '/yolo_detections_2d'

    return LaunchDescription(
        [
            Node(
                package='camera',
                executable='yolo_detection',
                output='screen',
                remappings=[
                    ('input_detection', input_topic),
                    ('output_detection_2d', output_topic),
                ],
            )
        ]
    )
