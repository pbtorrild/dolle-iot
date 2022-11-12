from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limcheck',
            namespace='sm2',
            executable='saveData',
            name='limcheck_1',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"device_path": "/dev/video0"},
                {"image_topic": "limcheck/cam1"}
            ]
        ),
        Node(
            package='limcheck',
            namespace='sm2',
            executable='saveData',
            name='limcheck_2',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"device_path": "/dev/video2"},
                {"image_topic": "limcheck/cam2"}
            ]
        )
    ])