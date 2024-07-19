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
