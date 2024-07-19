from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_common',
            namespace='chatbot',
            executable='audio_player_node',
            name='audio'
        )
    ])
