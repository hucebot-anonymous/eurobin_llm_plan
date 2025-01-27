docker run \
     --rm \
     --interactive \
     --tty \
     --runtime=nvidia \
     --gpus=all \
     --network=host \
     --device=/dev/snd:/dev/snd \
     --device=/dev/dri:/dev/dri \
     --name llm_whisper \
     --volume $(pwd)/src:/catkin_ws/src \
     --volume $(pwd)/models:/catkin_ws/models \
     llm_whisper
