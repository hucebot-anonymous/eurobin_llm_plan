Hostip="$(ip -4 -o a | awk '{print $4}' | cut -d/ -f1 | grep -v 127.0.0.1 | head -n1)"

docker run \
    --privileged \
    --rm \
    --interactive \
    --tty \
    --runtime=nvidia \
    --gpus=all \
    --network=host \
    --device=/dev/snd:/dev/snd \
    --device=/dev/dri:/dev/dri \
    --name speech_gen \
    --volume $(pwd)/scripts:/catkin_ws/scripts \
    --volume $(pwd)/src:/catkin_ws/src \
    --volume $(pwd)/models:/root/.local/share/ \
    speech_gen
