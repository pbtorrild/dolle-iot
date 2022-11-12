from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limcheck',
            namespace='sm2',
            executable='gpioTest',
            name='gpio_test',
            output="screen",
            emulate_tty=True
        )
    ])