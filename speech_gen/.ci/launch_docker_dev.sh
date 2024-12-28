# Define IP/Hostname pairs as an associative array
declare -A host_ips

host_ips=(
    ["tiago-61c"]="192.168.50.162"
    ["hucebot2024notebook"]="192.168.50.77"
    ["hucebot-2024-gpu-a"]="192.168.50.221"
    ["tiago"]="192.168.50.33"
    ["tiago-61c"]="10.68.0.1"
    ["teleop"]="192.168.50.63"
    ["dtotsila-laptop"]="192.168.50.118"
)

# Get the current hostname
current_hostname=$(hostname)

# Set ROS_IP based on the current hostname
ROS_IP=${host_ips[$current_hostname]}

# Check if ROS_IP was found for the hostname
if [ -z "$ROS_IP" ]; then
    echo "Error: No IP found for hostname $current_hostname"
    exit 1
fi

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
    --volume $(pwd)/speech_gen/:/catkin_ws/src/speech_gen \
    --volume $(pwd)/models:/root/.local/share/ \
    --env="ROS_IP=$ROS_IP" \
    --env="ROS_HOSTNAME=$ROS_IP" \
    speech_gen