from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tts_ros',
            namespace="chatbot",
            executable='tts_node',
            name='tts'
        )
    ])
