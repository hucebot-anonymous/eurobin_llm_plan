docker run \
     --rm \
     --interactive \
     --tty \
     --runtime=nvidia \
     --gpus=all \
     --network=host \
     --device=/dev/snd:/dev/snd \
     --device=/dev/dri:/dev/dri \
     --name fast_whisper \
     fast_whisper
