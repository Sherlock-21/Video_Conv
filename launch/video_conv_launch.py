import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for input and output topics
        DeclareLaunchArgument('input_topic', default_value='/usb_cam/image_raw', description='Input image topic'),
        DeclareLaunchArgument('output_topic', default_value='/converted_image', description='Output image topic'),
        DeclareLaunchArgument('framerate', default_value='30.0', description='Framerate for the camera'),

        # Launch the usb_cam node with framerate parameter
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            parameters=[{"framerate": launch.substitutions.LaunchConfiguration('framerate')}],
            remappings=[("image_raw", launch.substitutions.LaunchConfiguration('input_topic'))]
        ),

        # Launch the image_conversion node with input and output topic parameters
        Node(
            package="video_conv",
            executable="image_conversion",
            name="image_conversion",
            parameters=[
                {
                    'input_topic': launch.substitutions.LaunchConfiguration('input_topic'),
                    'output_topic': launch.substitutions.LaunchConfiguration('output_topic')
                }
            ],
            remappings=[
                ("input_image", launch.substitutions.LaunchConfiguration('input_topic')),
                ("output_image", launch.substitutions.LaunchConfiguration('output_topic'))
            ]
        )
    ])

