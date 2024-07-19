from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # a launcher for the chatbot's input and output
    return LaunchDescription([
        Node(
            package='chat',
            namespace='chatbot',
            executable='diction',
            name='chat_publisher',
            output='screen'
        ),
        Node(
            package='chat',
            namespace='chatbot',
            executable='listener',
            name='chat_listener',
            output='screen',
            parameters=[{"usedAI": "openai"}] # can be "anthropic" or "openai"
        )
    ])
