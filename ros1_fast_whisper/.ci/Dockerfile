FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ARG CUDNN_ARCH="linux-x86_64"

ENV TZ "Europe/Paris"

# Set the default shell to bash instead of sh
SHELL ["/bin/bash", "-c"]

ENV SHELL /bin/bash
ENV TERM xterm

# Install apt dependencies
RUN apt update && \
    apt install -y \
    python3 \
    python3-pip \
    python-is-python3 \
    sudo \
    terminator

# Install a newer version of CMake
RUN apt update && \
    apt install -y software-properties-common && \
    apt update && \
    apt install -y cmake


WORKDIR /temp

# Install ROS1
COPY .ci/install_ros1.sh /temp/install_ros1.sh
RUN chmod +x /temp/install_ros1.sh && \
    /temp/install_ros1.sh && \
    cd .. && \
    rm -rf /temp



# install cuda keyring
RUN apt update && \
    apt install -y wget && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb

# install apt dependencies
RUN apt update && \
    # libportaudio2 (for python-sounddevice)
    apt install -y libportaudio2 && \
    # cuda (also ship the required libcublas)
    apt install -y cuda-toolkit

# install cudnn8 from tarball
RUN wget https://developer.download.nvidia.com/compute/cudnn/redist/cudnn/${CUDNN_ARCH}/cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive.tar.xz && \
    tar -xvf cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive.tar.xz && \
    mv cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive/include/* /usr/local/cuda/include/ && \
    mv cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive/lib/* /usr/local/cuda/lib64/

# install python dependencies
RUN pip install faster-whisper sounddevice

WORKDIR /catkin_ws/models/

# download large-v3 model
RUN mkdir model && \
    wget https://huggingface.co/Systran/faster-whisper-large-v3/resolve/main/config.json -P model/ && \
    wget https://huggingface.co/Systran/faster-whisper-large-v3/resolve/main/preprocessor_config.json -P model/ && \
    wget https://huggingface.co/Systran/faster-whisper-large-v3/resolve/main/tokenizer.json -P model/ && \
    wget https://huggingface.co/Systran/faster-whisper-large-v3/resolve/main/vocabulary.json -P model/ && \
    wget https://huggingface.co/Systran/faster-whisper-large-v3/resolve/main/model.bin -P model/


WORKDIR /catkin_ws/src/

# Copy the command_generator package
# ADD fast_whisper_pkg/ /catkin_ws/src/fast_whisper_pkg/

WORKDIR /catkin_ws

ENV PATH="/opt/ros/noetic/bin:${PATH}"

COPY src/ /catkin_ws/src/
# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec bash"]
