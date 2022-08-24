from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limcheck',
            namespace='sm2',
            executable='camera',
            name='limcheck_1',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"camera"   :   "2"}
            ]
        ),
        Node(
            package='limcheck',
            namespace='sm2',
            executable='camera',
            name='limcheck_0',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"camera"   :   "0"}
            ]
        )        
    ])