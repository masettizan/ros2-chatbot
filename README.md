# Chatbot
created 07/12/2024 - 07/19/2024

## Installation

```shell
# clone
$ cd ~/ros2_ws/src
$ git clone https://github.com/emilytaylor6/chatbot.git

# dependencies
$ sudo apt-get install -y python-dev libportaudio2 libportaudiocpp0 portaudio19-dev libasound-dev swig
$ sudo apt install portaudio19-dev
$ cd chatbot
$ pip3 install -r audio_common/requirements.txt
$ pip3 install -r speech_to_text/requirements.txt
$ python3 ./nltk_download.py

# colcon
$ cd ~/ros2_ws
$ rosdep install -i --from-path src --rosdistro humble -y
$ colcon build
```

## Running
To run, write each command in different terminals after sourcing the workspace. 

```shell
$ ros2 launch speech_to_text speech_to_text.launch.py
```

```shell
$ ros2 run audio_common tts_node
```

```shell
$ ros2 run audio_common audio_player_node
```

```shell
$ ros2 launch chat chat_launch.launch.py
```

The default launch configuration for the chatbot uses openai, but it also supports anthropic within its parameters.

## Contained Repositories
Edited the following repositories for this chatbot:
- https://github.com/MERLIN2-ARCH/speech_to_text
- https://github.com/mgonzs13/audio_common
