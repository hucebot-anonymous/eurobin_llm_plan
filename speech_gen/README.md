# Speech Generation System

This project provides a robust pipeline for generating synthetic speech from text. The system listens to a ROS topic for text input, processes it using a text-to-speech (TTS) module, and publishes the resulting audio data to another ROS topic.

## Features
- Seamless text-to-speech conversion.
- Support for multiple TTS models and languages.
- ROS1 integration for easy deployment in robotic systems.
- Dockerized environment for simplified setup and development.

## Directory Structure
```
.
├── .ci
│   ├── Dockerfile                # Docker configuration for development
│   ├── install_ros1.sh           # Script to install ROS1
│   └── launch_docker_dev.sh      # Script to launch Docker container
├── models
│   └── tts                       # Pre-trained TTS models
├── README.md                     # Project documentation
├── speech_gen
│   ├── models                    # Models specific to speech generation
│   ├── README.md                 # Module-specific documentation
│   └── src
│       └── speech_gen_pkg        # ROS package for speech generation
│           └── scripts
│               └── speech_gen.py # Main Python script for text-to-speech
```
