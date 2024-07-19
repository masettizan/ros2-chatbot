import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sttLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('speech_to_text')),
            '/speech_to_text.launch.py'])
    )
    chatLaunches = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('chat'), 'launch'),
            '/chat_launch.launch.py'])
    )
    ttsLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tts_ros'), 'launch'),
            '/tts_launch.launch.py'])
    )
    audioLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('audio_common'), 'launch'),
            '/audio_common_launch.launch.py'])
    )

    return LaunchDescription([
        sttLaunch,
        chatLaunches,
        ttsLaunch,
        audioLaunch
    ])
