from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limcheck',
            namespace='stigemaskine_1',
            executable='limcheck',
            name='limcheck_1'
        ),
        Node(
            package='v4l2_camera',
            namespace='stigemaskine_1',
            executable='v4l2_camera_node',
            name='camera_1'
        )
    ])